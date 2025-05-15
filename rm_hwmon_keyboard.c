// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  System Voltage sensor platform driver for reMarkable HWMON
 */

#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/input/matrix_keypad.h>
#include <linux/backlight.h>
#include <linux/sysfs.h>

#include "rm_hwmon_api.h"
#include "rm_hwmon_fwu.h"

#define FWU_MAX_PACKET_SIZE 256

#define MAX_BL_BRIGHTNESS 255
#define ATTRIBUTES_NR_OF_BKLS 6

/**
 * enum condor_language - keyboard language
 *
 * This list is from condor_mcu repo -> source/attributes.h
 * Copied 5. February 2024
 */
enum condor_language {
	LANGUAGE_MIN = 0,
	DE,
	ES,
	FR,
	IT,
	NO,
	PT,
	UK,
	US,
	LANGUAGE_MAX
};

/**
 * enum condor_attribute_ids - attributes in Condor
 *
 * This list is from condor_mcu repo -> source/attributes.h
 * Copied 10. November 2023
 */
enum condor_attribute_ids {
	/* Read only attributes */
	KB_ATTR_ID_PROTOCOL_VERSION = 0x01,
	KB_ATTR_ID_FW_VERSION,
	KB_ATTR_ID_HW_VERSION,
	KB_ATTR_ID_DEVICE_CLASS,
	KB_ATTR_ID_DEVICE_ID,
	KB_ATTR_ID_IMAGE_START_ADDRESS,
	KB_ATTR_ID_DEVICE_NAME,
	KB_ATTR_ID_GIT_INFO,
	KB_ATTR_ID_VALID_IMAGE,

	/* Read/Write attributes */
	KB_ATTR_ID_KEY_LAYOUT = 0x10,
	KB_ATTR_ID_LANGUAGE,
	KB_ATTR_ID_RM_SERIAL_NUMBER,
	KB_ATTR_ID_CN_SERIAL_NUMBER,

	/* Production and test attributes */
	KB_ATTR_ID_MFG_PROD_RECORDS = 0x20,

	/* Communication delays */
	KB_ATTR_ID_ALIVE_MESSAGE_TIMEOUT_MS = 0x30,
	KB_ATTR_ID_MATRIX_SCAN_DELAY_US,

	/* Keyboard debouncing params */
	KB_ATTR_ID_KEY_DEBOUNCE_TIME_MS = 0x40,
	KB_ATTR_ID_DEBOUNCE_TIME_PRECISION_MS,

	/* Backlight attributes */
	KB_ATTR_ID_BACKLIGHT_RANGE = 0x50,
	KB_ATTR_ID_BKL_COEFF,
	KB_ATTR_ID_BKL_BRIGHTNESS,
	KB_ATTR_ID_KEY_LIGHT_CAPS,
	KB_ATTR_ID_KEY_LIGHT_RM,
};

/**
 * struct kb_attr_write - Structure for writing attribute and not checking answer
 * @worker: Work thread
 * @request: The request to send. Will be freed when done
 * @attribute: Attribute to write to
 * @data_length: Length of data to send
 */
struct kb_attr_write {
	struct work_struct worker;
	struct attribute_tx *request;
	uint8_t attribute;
	uint32_t data_length;
};

/**
 * struct kb_data - System struct for keyboard
 * @dev: Basic device structure
 * @parent_dev: Parent device which have rm_hwmon struct
 * @kb_dev: Input device, if NULL input is not available
 * @kb_connect_work: Work thread
 * @kb_connect_lock: Mutex for keyboard connect and disconnect events
 * @attr_writer: Attribute writer for interrupts and atomic operations
 * @kb_row_shift: Number of keyboard columns
 * @key_layout: Which layout should be used in ::keymap_layouts
 * @language: Which language the keyboard have on keys
 * @rm_serial_number: ReMarkable serial number
 * @cn_serial_number: Chicony serial number
 * @mfg_prod_records: Production records is only used for manufacturing
 * @device_name: Name of keyboard
 * @git_info: Git hash and 1 bit that tells if it is dirty or not
 * @bl_brightness: Brightness we want to set on keyboard
 * @bl_brightness_array: Brightness values that are sent to all LEDs in keyboard
 * @is_image_valid: Used for FWU to see if image is validated or not
 * @rm_key_light: Status of rM key light
 * @caps_key_ligh: Status of CAPS key light
 * @fwu: Firmware structure used for FWU
 */
struct kb_data {
	struct device *dev;
	struct device *parent_dev;

	struct input_dev *kb_dev;
	struct work_struct kb_connect_work;
	struct mutex kb_connect_lock;

	struct kb_attr_write attr_writer;
	int kb_row_shift;

	uint8_t key_layout;
	uint8_t language;
	char *rm_serial_number;
	char *cn_serial_number;
	uint8_t mfg_prod_records;
	char *device_name;
	uint32_t git_info;

	int bl_brightness;
	uint8_t bl_brightness_array[ATTRIBUTES_NR_OF_BKLS];
	bool is_image_valid;
	bool rm_key_light;
	bool caps_key_light;
	bool rm_key_on_after_resume;

	struct firmware_update fwu;
};

/**
 * struct rm_hwmon_kb_keymap_data - Internal structure for ordering the supported keyboard layouts
 */
struct rm_hwmon_kb_keymap_data {
	struct matrix_keymap_data keymap_data;
	u8 row;
	u8 col;
};

/**
 * struct rm_hwmon_kb_key_event - Structure for parsing a key event from keyboard
 */
struct __packed rm_hwmon_kb_key_event {
	uint8_t pressed : 1;
	uint8_t row : 3;
	uint8_t column : 4;
	uint8_t seq_num;
};

MAKE_READER(key_layout, u8_data, struct kb_data, key_layout)
MAKE_READER(language, u8_data, struct kb_data, language)
MAKE_READER(image_start_address, u32_data, struct kb_data, fwu.current_image_start_address)
MAKE_READER(git_info, u32_data, struct kb_data, git_info)
MAKE_READER(valid_image, bool_data, struct kb_data, is_image_valid)
MAKE_READER(prod_records, u8_data, struct kb_data, mfg_prod_records)
MAKE_READER(rm_key_light, bool_data, struct kb_data, rm_key_light)
MAKE_READER(caps_key_light, bool_data, struct kb_data, caps_key_light)

static bool reader_fw_version(void *client_data,
			      const struct attribute_rx *const attr_res)
{
	struct kb_data *pdata = (struct kb_data *)client_data;

	pdata->fwu.current_fw_version = *((struct fw_version *)&attr_res->data.u16_data);
	return true;
}

static bool reader_bl_brightness_array(void *client_data,
				       const struct attribute_rx *const attr_res)
{
	int i;
	struct kb_data *pdata = (struct kb_data *)client_data;

	if (attr_res->data.array.subtype != ATTRIBUTE_STORAGE_DATA_UNSIGNED_8_BIT)
		return false;

	if (attr_res->data.array.n_len != ATTRIBUTES_NR_OF_BKLS)
		return false;

	for (i = 0; i < ATTRIBUTES_NR_OF_BKLS; i++)
		pdata->bl_brightness_array[i] = attr_res->data.array.array_data[i];

	return true;
}

static bool reader_octet_attribute(void *client_data,
				   const struct attribute_rx *const attr_res)
{
	struct kb_data *pdata = (struct kb_data *)client_data;
	char **octet_attribute = NULL;

	switch (attr_res->id) {
	case KB_ATTR_ID_DEVICE_NAME:
		octet_attribute = &pdata->device_name;
		break;

	case KB_ATTR_ID_RM_SERIAL_NUMBER:
		octet_attribute = &pdata->rm_serial_number;
		break;

	case KB_ATTR_ID_CN_SERIAL_NUMBER:
		octet_attribute = &pdata->cn_serial_number;
		break;

	default:
		return false;
	}

	*octet_attribute = devm_krealloc(pdata->dev,
					 *octet_attribute,
					 attr_res->data.octets.str_len + 1,
					 GFP_KERNEL);

	if (*octet_attribute) {
		memcpy(*octet_attribute, attr_res->data.octets.str,
		       attr_res->data.octets.str_len);
		(*octet_attribute)[attr_res->data.octets.str_len] = '\0';

		return true;
	}

	return false;
}

/*
 * List of some of the attributes supported in Condor
 */
static struct attribute_config attribute_configs[] = {
	{	.id = KB_ATTR_ID_KEY_LAYOUT,
		.type = ATTRIBUTE_STORAGE_DATA_UNSIGNED_8_BIT,
		.reader = reader_key_layout,
		.name = "KB_ATTR_ID_KEY_LAYOUT"
	},
	{	.id = KB_ATTR_ID_LANGUAGE,
		.type = ATTRIBUTE_STORAGE_DATA_ENUM_8_BIT,
		.reader = reader_language,
		.name = "KB_ATTR_ID_LANGUAGE"
	},
	{	.id = KB_ATTR_ID_RM_SERIAL_NUMBER,
		.type = ATTRIBUTE_STORAGE_DATA_CHARACTER_STRING,
		.reader = reader_octet_attribute,
		.name = "KB_ATTR_ID_RM_SERIAL_NUMBER"
	},
	{	.id = KB_ATTR_ID_CN_SERIAL_NUMBER,
		.type = ATTRIBUTE_STORAGE_DATA_CHARACTER_STRING,
		.reader = reader_octet_attribute,
		.name = "KB_ATTR_ID_CN_SERIAL_NUMBER"
	},
	{	.id = KB_ATTR_ID_MFG_PROD_RECORDS,
		.type = ATTRIBUTE_STORAGE_DATA_UNSIGNED_8_BIT,
		.reader = reader_prod_records,
		.name = "KB_ATTR_ID_MFG_PROD_RECORDS"
	},
	{	.id = KB_ATTR_ID_FW_VERSION,
		.type = ATTRIBUTE_STORAGE_DATA_UNSIGNED_16_BIT,
		.reader = reader_fw_version,
		.name = "KB_ATTR_ID_FW_VERSION"
	},
	{	.id = KB_ATTR_ID_IMAGE_START_ADDRESS,
		.type = ATTRIBUTE_STORAGE_DATA_UNSIGNED_32_BIT,
		.reader = reader_image_start_address,
		.name = "KB_ATTR_ID_IMAGE_START_ADDRESS"
	},
	{	.id = KB_ATTR_ID_DEVICE_NAME,
		.type = ATTRIBUTE_STORAGE_DATA_CHARACTER_STRING,
		.reader = reader_octet_attribute,
		.name = "KB_ATTR_ID_DEVICE_NAME"
	},
	{	.id = KB_ATTR_ID_GIT_INFO,
		.type = ATTRIBUTE_STORAGE_DATA_32_BIT,
		.reader = reader_git_info,
		.name = "KB_ATTR_ID_GIT_INFO"
	},
	{	.id = KB_ATTR_ID_BKL_BRIGHTNESS,
		.type = ATTRIBUTE_STORAGE_DATA_ARRAY,
		.reader = reader_bl_brightness_array,
		.name = "KB_ATTR_ID_BKL_BRIGHTNESS"
	},
	{	.id = KB_ATTR_ID_VALID_IMAGE,
		.type = ATTRIBUTE_STORAGE_DATA_BOOLEAN,
		.reader = reader_valid_image,
		.name = "KB_ATTR_ID_VALID_IMAGE"
	},
	{	.id = KB_ATTR_ID_KEY_LIGHT_CAPS,
		.type = ATTRIBUTE_STORAGE_DATA_BOOLEAN,
		.reader = reader_rm_key_light,
		.name = "KB_ATTR_ID_KEY_LIGHT_CAPS"
	},
	{	.id = KB_ATTR_ID_KEY_LIGHT_RM,
		.type = ATTRIBUTE_STORAGE_DATA_BOOLEAN,
		.reader = reader_caps_key_light,
		.name = "KB_ATTR_ID_KEY_LIGHT_RM"
	},
};

/**
 * Keymap table for Condor v1
 * 2023.07.04
 */
static const uint32_t rm_hwmon_keymap_v1[] = {
	KEY(0, 0, KEY_M),
	KEY(0, 1, KEY_N),
	KEY(0, 2, KEY_EQUAL),
	KEY(0, 3, KEY_RESERVED),
	KEY(0, 4, KEY_GRAVE),
	KEY(0, 5, KEY_EQUAL),
	KEY(0, 6, KEY_RESERVED),
	KEY(0, 7, KEY_A),
	KEY(0, 8, KEY_RESERVED),
	KEY(0, 9, KEY_RESERVED),
	KEY(0, 10, KEY_3),
	KEY(0, 11, KEY_Y),
	KEY(0, 12, KEY_O),
	KEY(0, 13, KEY_RESERVED),
	KEY(0, 14, KEY_R),
	KEY(0, 15, KEY_RESERVED),
	KEY(1, 0, KEY_9),
	KEY(1, 1, KEY_RESERVED),
	KEY(1, 2, KEY_L),
	KEY(1, 3, KEY_RESERVED),
	KEY(1, 4, KEY_MINUS),
	KEY(1, 5, KEY_RESERVED),
	KEY(1, 6, KEY_DOT),
	KEY(1, 7, KEY_SLASH),
	KEY(1, 8, KEY_RESERVED),
	KEY(1, 9, KEY_RESERVED),
	KEY(1, 10, KEY_RESERVED),
	KEY(1, 11, KEY_END),
	KEY(1, 12, KEY_RESERVED),
	KEY(1, 13, KEY_SEMICOLON),
	KEY(1, 14, KEY_RESERVED),
	KEY(1, 15, KEY_RESERVED),
	KEY(2, 0, KEY_RESERVED),
	KEY(2, 1, KEY_RESERVED),
	KEY(2, 2, KEY_RESERVED),
	KEY(2, 3, KEY_RESERVED),
	KEY(2, 4, KEY_RIGHTMETA),
	KEY(2, 5, KEY_2),
	KEY(2, 6, KEY_RESERVED),
	KEY(2, 7, KEY_UP),
	KEY(2, 8, KEY_LEFTALT),
	KEY(2, 9, KEY_LEFT),
	KEY(2, 10, KEY_APOSTROPHE),
	KEY(2, 11, KEY_RESERVED),
	KEY(2, 12, KEY_RESERVED),
	KEY(2, 13, KEY_F),
	KEY(2, 14, KEY_ENTER),
	KEY(2, 15, KEY_RESERVED),
	KEY(3, 0, KEY_RESERVED),
	KEY(3, 1, KEY_B),
	KEY(3, 2, KEY_RESERVED),
	KEY(3, 3, KEY_LEFTSHIFT),
	KEY(3, 4, KEY_RESERVED),
	KEY(3, 5, KEY_6),
	KEY(3, 6, KEY_C),
	KEY(3, 7, KEY_RESERVED),
	KEY(3, 8, KEY_SPACE),
	KEY(3, 9, KEY_RESERVED),
	KEY(3, 10, KEY_X),
	KEY(3, 11, KEY_4),
	KEY(3, 12, KEY_U),
	KEY(3, 13, KEY_D),
	KEY(3, 14, KEY_RESERVED),
	KEY(3, 15, KEY_RESERVED),
	KEY(4, 0, KEY_W),
	KEY(4, 1, KEY_CAPSLOCK),
	KEY(4, 2, KEY_Z),
	KEY(4, 3, KEY_RIGHTSHIFT),
	KEY(4, 4, KEY_5),
	KEY(4, 5, KEY_RESERVED),
	KEY(4, 6, KEY_T),
	KEY(4, 7, KEY_RIGHT),
	KEY(4, 8, KEY_RESERVED),
	KEY(4, 9, KEY_DOWN),
	KEY(4, 10, KEY_RESERVED),
	KEY(4, 11, KEY_RESERVED),
	KEY(4, 12, KEY_K),
	KEY(4, 13, KEY_RESERVED),
	KEY(4, 14, KEY_RIGHTALT),
	KEY(4, 15, KEY_J),
	KEY(5, 0, KEY_G),
	KEY(5, 1, KEY_RESERVED),
	KEY(5, 2, KEY_8),
	KEY(5, 3, KEY_RESERVED),
	KEY(5, 4, KEY_ESC),
	KEY(5, 5, KEY_0),
	KEY(5, 6, KEY_S),
	KEY(5, 7, KEY_RESERVED),
	KEY(5, 8, KEY_RESERVED),
	KEY(5, 9, KEY_RESERVED),
	KEY(5, 10, KEY_E),
	KEY(5, 11, KEY_RESERVED),
	KEY(5, 12, KEY_V),
	KEY(5, 13, KEY_I),
	KEY(5, 14, KEY_COMMA),
	KEY(5, 15, KEY_LEFTCTRL),
	KEY(6, 0, KEY_H),
	KEY(6, 1, KEY_Q),
	KEY(6, 2, KEY_RESERVED),
	KEY(6, 3, KEY_RESERVED),
	KEY(6, 4, KEY_7),
	KEY(6, 5, KEY_MINUS),
	KEY(6, 6, KEY_BACKSLASH),
	KEY(6, 7, KEY_RESERVED),
	KEY(6, 8, KEY_BACKSPACE),
	KEY(6, 9, KEY_RESERVED),
	KEY(6, 10, KEY_P),
	KEY(6, 11, KEY_1),
	KEY(6, 12, KEY_RESERVED),
	KEY(6, 13, KEY_TAB),
	KEY(6, 14, KEY_RESERVED),
	KEY(6, 15, KEY_RESERVED)
};

static const struct rm_hwmon_kb_keymap_data keymap_layouts[] = {
	{
		.keymap_data = {
			.keymap = rm_hwmon_keymap_v1,
			.keymap_size = ARRAY_SIZE(rm_hwmon_keymap_v1),
		},
		.row = 7,
		.col = 16,
	}
};

/**
 * rm_hwmon_keyboard_read_init_attributes() - Read out attribute for initialization
 * @pdata: keyboard data structure
 *
 * Return: 0 on success, negative numer otherwise.
 */
static int rm_hwmon_keyboard_read_init_attributes(struct kb_data *pdata)
{
	const uint8_t init_attr[] = {KB_ATTR_ID_KEY_LAYOUT, KB_ATTR_ID_LANGUAGE,
				     KB_ATTR_ID_FW_VERSION, KB_ATTR_ID_GIT_INFO,
				     KB_ATTR_ID_IMAGE_START_ADDRESS, KB_ATTR_ID_DEVICE_NAME,
				     KB_ATTR_ID_VALID_IMAGE, KB_ATTR_ID_RM_SERIAL_NUMBER};

	return rm_hwmon_api_read_attributes(pdata->parent_dev, ENDPOINT_KEYBOARD,
					    init_attr, ARRAY_SIZE(init_attr));
}

/**
 * rm_hwmon_keyboard_update_fwu() - Do complete firmware update of accessory
 * @pdata: keyboard data structure
 *
 * Return: 0 on success, negative numer otherwise.
 */
static int rm_hwmon_keyboard_update_fwu(struct kb_data *pdata)
{
	int ret;

	ret = rm_hwmon_fwu_init(pdata->parent_dev, &pdata->fwu);
	if (ret) {
		dev_warn(pdata->dev, "%s: FWU Init failed with error %d\n", __func__, ret);
		return ret;
	}

	ret = rm_hwmon_fwu_transfer_binary(pdata->parent_dev, &pdata->fwu);
	if (ret) {
		dev_warn(pdata->dev, "%s: FWU send binary failed with error %d\n", __func__, ret);
		return ret;
	}

	ret = rm_hwmon_fwu_validate_image(pdata->parent_dev, &pdata->fwu);
	if (ret) {
		dev_warn(pdata->dev, "%s: FWU validate image failed with error %d\n",
			 __func__, ret);
		return ret;
	}

	ret = rm_hwmon_keyboard_read_init_attributes(pdata);
	if (ret) {
		dev_warn(pdata->dev, "%s: FWU read initial attributes failed with error %d\n",
			 __func__, ret);
		return ret;
	}

	ret = memcmp(&pdata->fwu.header.fw_version, &pdata->fwu.current_fw_version,
		     sizeof(struct fw_version));
	if (ret) {
		dev_warn(pdata->dev, "%s: FWU Failed keyboard and binary firmware do not match\n",
			  __func__);
		return ret;
	}

	ret = rm_hwmon_fwu_set_image_active(pdata->parent_dev, &pdata->fwu);

	if (!ret)
		dev_info(pdata->dev, "%s: Firmware upgraded successful to version %d.%d\n",
			 __func__, pdata->fwu.current_fw_version.major,
			 pdata->fwu.current_fw_version.minor);

	return ret;
}

/**
 * rm_hwmon_keyboard_set_brightness() - Set keyboard brightness
 * @pdata: Keyboard internal structure
 *
 * Return: 0 on success and a negative number on failure.
 */
static int rm_hwmon_keyboard_set_brightness(struct kb_data *pdata)
{
	int ret, i;
	const uint32_t data_length = HEADER_SIZE_ARRAY + sizeof(uint8_t) * ATTRIBUTES_NR_OF_BKLS;
	struct attribute_tx *write_req = rm_hwmon_api_alloc_write_buffer(pdata->dev, data_length);

	if (!write_req)
		return -ENOMEM;

	write_req->data.array.subtype = ATTRIBUTE_STORAGE_DATA_UNSIGNED_8_BIT;
	write_req->data.array.n_len = ATTRIBUTES_NR_OF_BKLS;

	for (i = 0; i < ATTRIBUTES_NR_OF_BKLS; i++)
		write_req->data.array.array_data[i] = (uint8_t)pdata->bl_brightness;

	ret = rm_hwmon_api_write_attribute(pdata->parent_dev, ENDPOINT_KEYBOARD,
					   KB_ATTR_ID_BKL_BRIGHTNESS, write_req, data_length);

	kfree(write_req);

	return ret;
}

static int rm_hwmon_kb_bl_update_status(struct backlight_device *bd)
{
	int brightness = backlight_get_brightness(bd);
	struct kb_data *pdata = bl_get_data(bd);
	int ret = 0;

	if (brightness != pdata->bl_brightness) {
		pdata->bl_brightness = brightness;

		/*
		 * Only send brightness if keyboard is connected.
		 * Brightness is sent on connection by rm_hwmon_keyboard_register.
		 */
		if (pdata->kb_dev)
			ret = rm_hwmon_keyboard_set_brightness(pdata);
	}

	return ret;
}

static const struct backlight_ops rm_hwmon_kb_bl_ops = {
	.update_status = rm_hwmon_kb_bl_update_status,
};

static int rm_hwmon_kb_event(struct input_dev *dev, unsigned int type,
			     unsigned int code, int value)
{
	struct kb_data *pdata = input_get_drvdata(dev);
	struct rm_hwmon_data *parent_pdata = dev_get_drvdata(pdata->parent_dev);

	if (type != EV_LED)
		return -1;

	switch (code) {
	case LED_CAPSL:
		pdata->attr_writer.attribute = KB_ATTR_ID_KEY_LIGHT_CAPS;
		break;

	case LED_MISC:
		pdata->attr_writer.attribute = KB_ATTR_ID_KEY_LIGHT_RM;
		break;

	default:
		return -1;
	}

	if (pdata->attr_writer.request)
		return -EBUSY;

	/* We don't want the condor LEDs to go off when we are in slumber */
	if (!value && parent_pdata->next_suspend_is_slumber)
		return 0;

	pdata->attr_writer.data_length = sizeof(uint8_t);
	pdata->attr_writer.request = rm_hwmon_api_alloc_write_buffer(pdata->dev,
								     sizeof(uint8_t));
	if (!pdata->attr_writer.request)
		return -ENOMEM;

	pdata->attr_writer.request->data.u8_data = (uint8_t)value;

	if (!schedule_work(&pdata->attr_writer.worker)) {
		kfree(pdata->attr_writer.request);
		pdata->attr_writer.request = NULL;
		return -1;
	}

	/* Attributes are never read out, set state manually */
	switch (code) {
	case LED_CAPSL:
		pdata->caps_key_light = value ? true : false;
		break;
	case LED_MISC:
		pdata->rm_key_light = value ? true : false;
		break;
	default:
		return -1;
	}

	return 0;
}

/**
 * rm_hwmon_keyboard_unregister() - Remove keyboard from system
 * @pdata: Keyboard internal structure
 *
 * Return: true if removed, otherwise false
 */
static bool rm_hwmon_keyboard_unregister(struct kb_data *pdata)
{
	if (pdata->kb_dev != NULL) {
		__clear_bit(LED_CAPSL, pdata->kb_dev->ledbit);
		__clear_bit(LED_MISC, pdata->kb_dev->ledbit);
		input_unregister_device(pdata->kb_dev);
		pdata->kb_dev = NULL;
		dev_info(pdata->dev, "RM HWMON keyboard removed\n");
		return true;
	}
	return false;
}

/**
 * rm_hwmon_keyboard_make_input_dev() - Helper function for making input dev
 * @pdata: Keyboard internal structure
 *
 * Return: 0 on success and a negative number on failure.
 */
static int rm_hwmon_keyboard_make_input_dev(struct kb_data *pdata)
{
	int ret;
	struct input_dev *kb_dev;
	const struct rm_hwmon_kb_keymap_data *keymap_layout;
	struct device *dev = pdata->dev;

	pdata->kb_dev = NULL;
	if (pdata->key_layout >= ARRAY_SIZE(keymap_layouts)) {
		dev_warn(dev, "Unknown keyboard layout (%d) configured. Set default version\n",
			 pdata->key_layout);
		pdata->key_layout = 0;
	}

	kb_dev = input_allocate_device();
	if (!kb_dev)
		return -EINVAL;

	keymap_layout = &keymap_layouts[pdata->key_layout];
	pdata->kb_dev = kb_dev;
	pdata->kb_row_shift = get_count_order(keymap_layout->col);
	kb_dev->name = "rM_Keyboard";
	kb_dev->phys = "pogo/input0";
	kb_dev->id.bustype = BUS_HOST;
	kb_dev->id.vendor  = 0x2edd;		/* TODO: rM vendor ID */
	kb_dev->id.product = 0x0001;		/* TODO: to be decided */
	kb_dev->id.version = 0x0100;
	kb_dev->uniq = pdata->rm_serial_number;
	kb_dev->dev.parent = dev;
	kb_dev->event = rm_hwmon_kb_event;

	input_set_drvdata(kb_dev, pdata);

	ret = matrix_keypad_build_keymap(&keymap_layout->keymap_data, NULL, keymap_layout->row,
					 keymap_layout->col, NULL, kb_dev);
	if (ret) {
		dev_err(dev, "Failed to build keymap\n");
		return ret;
	}

	/* setup kb_dev device */
	__set_bit(EV_KEY, kb_dev->evbit);
	__set_bit(EV_REP, kb_dev->evbit);
	input_set_capability(kb_dev, EV_LED, LED_CAPSL);
	input_set_capability(kb_dev, EV_LED, LED_MISC);

	ret = input_register_device(kb_dev);
	if (ret) {
		dev_err(dev, "Failed register input device\n");
		return ret;
	}

	return 0;
}

/**
 * rm_hwmon_keyboard_register() - Register keyboard as a keyboard device
 * @pdata: Keyboard internal structure
 *
 * Return: 0 on success and a negative number on failure.
 */
static int rm_hwmon_keyboard_register(struct kb_data *pdata)
{
	int ret;
	bool auth_successful = false;
	bool fw_upgraded = false;
	bool input_dev_ok = false;
	struct device *dev = pdata->dev;

	mutex_lock(&pdata->kb_connect_lock);

	if (rm_hwmon_keyboard_unregister(pdata))
		dev_warn(dev, "No disconnect event before new connect\n");

	pdata->rm_key_light = false;
	pdata->caps_key_light = false;
	pdata->rm_key_on_after_resume = false;

	ret = rm_hwmon_keyboard_read_init_attributes(pdata);
	if (ret)
		goto unlock_mutex;

	ret = rm_hwmon_api_write_cmd(pdata->parent_dev, ENDPOINT_KEYBOARD,
				     HSP_CMD_ACCS_AUTHORIZE_REQUEST, NULL, 0);
	if (ret) {
		dev_err(dev, "Authorization failed. Try FWU\n");
	} else {
		ret = rm_hwmon_keyboard_make_input_dev(pdata);
		if (ret)
			goto kb_fail_register;

		input_dev_ok = true;
		auth_successful = true;

		ret = rm_hwmon_keyboard_set_brightness(pdata);
		if (ret)
			dev_warn(dev, "Failed to set keyboard brightness\n");
	}

	if (rm_hwmon_fwu_load_and_check_for_upgrade(pdata->dev, &pdata->fwu, pdata->device_name)) {
		ret = rm_hwmon_keyboard_update_fwu(pdata);
		rm_hwmon_fwu_release_firmware(pdata->dev, &pdata->fwu);
		if (ret)
			dev_warn(pdata->dev,
				 "%s: FWU of keyboard failed with error %d. Continue with existing version\n",
				 __func__, ret);
		else
			fw_upgraded = true;
	}

	if (fw_upgraded) {
		ret = rm_hwmon_api_write_cmd(pdata->parent_dev, ENDPOINT_KEYBOARD,
					     HSP_CMD_ACCS_AUTHORIZE_REQUEST, NULL, 0);
		if (ret) {
			dev_err(dev, "Authorization failed. Abort keyboard setup\n");
			goto kb_fail_register;
		}

		ret = rm_hwmon_keyboard_set_brightness(pdata);
		if (ret)
			dev_warn(dev, "Failed to set keyboard brightness\n");

	} else if (!auth_successful) {
		dev_err(dev, "Authorization failed. Abort keyboard setup\n");
		goto kb_fail_register;
	}

	if (!input_dev_ok) {
		ret = rm_hwmon_keyboard_make_input_dev(pdata);
		if (ret)
			goto kb_fail_register;
	}

	dev_info(dev, "Device %s registered. FW: %d.%d, SHA: %07x%s\n",
		 pdata->kb_dev->name,
		 pdata->fwu.current_fw_version.major, pdata->fwu.current_fw_version.minor,
		 pdata->git_info >> 4, (pdata->git_info & 0xf) ? "-dirty" : "");

	goto unlock_mutex;

kb_fail_register:
	if (pdata->kb_dev) {
		__clear_bit(LED_CAPSL, pdata->kb_dev->ledbit);
		__clear_bit(LED_MISC, pdata->kb_dev->ledbit);
		input_unregister_device(pdata->kb_dev);
	}
	pdata->kb_dev = NULL;

unlock_mutex:
	mutex_unlock(&pdata->kb_connect_lock);
	return ret;

}

/**
 * rm_hwmon_keyboard_report() - Report key for linux system
 * @pdata: Keyboard internal structure
 * @event: Key event from HWMON
 *
 * Transform event to linux key with the help of key layout register for input device
 *
 * Return: 0 on success and a negative number on failure.
 */
static int rm_hwmon_keyboard_report(struct kb_data *pdata, struct rm_hwmon_kb_key_event *event)
{
	int key;
	const unsigned short *keymap = NULL;

	if (pdata->kb_dev == NULL) {
		dev_warn(pdata->dev, "Connect event need to be sent before key events\n");
		schedule_work(&pdata->kb_connect_work);
		return -ENXIO;
	}

	keymap = pdata->kb_dev->keycode;
	key = MATRIX_SCAN_CODE(event->row, event->column, pdata->kb_row_shift);

	dev_dbg(pdata->dev, "Report row %d column %d key_idx %d code %d active %d\n",
		event->row, event->column, key, keymap[key], event->pressed);

	input_report_key(pdata->kb_dev, keymap[key], event->pressed);
	input_sync(pdata->kb_dev);
	return 0;
}

/**
 * rm_hwmon_keyboard_hsp_event() - Callback for messages sent from keyboard
 * @dev: Keyboard device structure
 * @packet: packet with data from keyboard
 *
 * Return: 0 on success and a negative number on failure.
 */
static int rm_hwmon_keyboard_hsp_event(struct device *dev, struct hsp_packet *packet)
{
	int ret = 0;
	struct kb_data *kb_data = dev_get_drvdata(dev);

	switch (packet->command) {
	case HSP_CMD_ACCS_CONNECT:
		schedule_work(&kb_data->kb_connect_work);
		break;

	case HSP_CMD_ACCS_DISCONNECT:
		mutex_lock(&kb_data->kb_connect_lock);
		rm_hwmon_keyboard_unregister(kb_data);
		mutex_unlock(&kb_data->kb_connect_lock);
		break;

	case HSP_CMD_KEY_EVENT:
		ret = rm_hwmon_keyboard_report(kb_data,
					       (struct rm_hwmon_kb_key_event *)packet->data);
		break;
	case HSP_CMD_ACCS_AUTHORIZE_REQUEST:
		ret = (packet->length == 1 && packet->data[0] == HSP_CMD_SUCCESS) ? 0 : -EACCES;
		rm_hwmon_api_req_complete(kb_data->parent_dev, ret);
		break;

	default:
		ret = -ENODEV;
		break;
	}

	return ret;
}

static void rm_hwmon_keyboard_connect_event(struct work_struct *work)
{
	struct kb_data *pdata = container_of(work, struct kb_data, kb_connect_work);

	rm_hwmon_keyboard_register(pdata);
}

static void rm_hwmon_keyboard_attribute_writer(struct work_struct *work)
{
	int ret;
	struct kb_data *pdata = container_of(work, struct kb_data, attr_writer.worker);

	if (pdata->attr_writer.request) {
		ret = rm_hwmon_api_write_attribute(pdata->parent_dev, ENDPOINT_KEYBOARD,
						pdata->attr_writer.attribute,
						pdata->attr_writer.request,
						pdata->attr_writer.data_length);

		if (ret)
			dev_warn(pdata->dev,
				 "%s: Failed to write attribute with id: %u and reason: %d\n",
				 __func__, pdata->attr_writer.attribute, ret);

		kfree(pdata->attr_writer.request);
		pdata->attr_writer.request = NULL;
	}
}

static ssize_t firmware_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct kb_data *pdata = dev_get_drvdata(dev);

	if (!pdata->kb_dev)
		return -ENODEV;

	return sprintf(buf, "%d.%d\n", pdata->fwu.current_fw_version.major,
		       pdata->fwu.current_fw_version.minor);
}

static ssize_t rm_serial_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct kb_data *pdata = dev_get_drvdata(dev);

	if (!pdata->kb_dev || !pdata->rm_serial_number)
		return -ENODEV;

	return sprintf(buf, "%s\n", pdata->rm_serial_number);
}

static ssize_t cn_serial_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	struct kb_data *pdata = dev_get_drvdata(dev);

	if (!pdata->kb_dev)
		return -ENODEV;

	ret = rm_hwmon_api_read_attribute(pdata->parent_dev, ENDPOINT_KEYBOARD,
					  KB_ATTR_ID_CN_SERIAL_NUMBER);
	if (ret)
		return ret;

	return sprintf(buf, "%s\n", pdata->cn_serial_number);
}

static ssize_t prod_records_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	struct kb_data *pdata = dev_get_drvdata(dev);

	if (!pdata->kb_dev)
		return -ENODEV;

	ret = rm_hwmon_api_read_attribute(pdata->parent_dev, ENDPOINT_KEYBOARD,
					  KB_ATTR_ID_MFG_PROD_RECORDS);
	if (ret)
		return ret;

	return sprintf(buf, "%x\n", pdata->mfg_prod_records);
}

/* Must be aligned with condor_language */
static const char * const sysfs_language[] = {
	"ILLEGAL",
	"DE",
	"ES",
	"FR",
	"IT",
	"NO",
	"PT",
	"UK",
	"US",
};

static ssize_t language_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct kb_data *pdata = dev_get_drvdata(dev);

	if (!pdata->kb_dev)
		return -ENODEV;

	if (pdata->language <= LANGUAGE_MIN || LANGUAGE_MAX <= pdata->language)
		return -ENOEXEC;

	return sprintf(buf, "%s\n", sysfs_language[pdata->language]);
}


static DEVICE_ATTR_RO(firmware);
static DEVICE_ATTR_RO(rm_serial);
static DEVICE_ATTR_RO(cn_serial);
static DEVICE_ATTR_RO(language);
static DEVICE_ATTR_RO(prod_records);

static struct attribute *dev_attrs[] = {
	&dev_attr_firmware.attr,
	&dev_attr_rm_serial.attr,
	&dev_attr_cn_serial.attr,
	&dev_attr_language.attr,
	&dev_attr_prod_records.attr,
	NULL,
};

ATTRIBUTE_GROUPS(dev);

static int rm_hwmon_keyboard_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct kb_data *kb_data = NULL;
	struct backlight_properties props;
	struct backlight_device *bd;

	kb_data = devm_kzalloc(dev, sizeof(struct kb_data), GFP_KERNEL);
	if (!kb_data)
		return -ENOMEM;

	kb_data->dev = dev;
	kb_data->parent_dev = dev->parent;

	kb_data->fwu.max_packet_size = FWU_MAX_PACKET_SIZE;
	kb_data->fwu.endpoint = ENDPOINT_KEYBOARD;

	dev_set_drvdata(dev, kb_data);
	INIT_WORK(&kb_data->kb_connect_work, rm_hwmon_keyboard_connect_event);
	INIT_WORK(&kb_data->attr_writer.worker, rm_hwmon_keyboard_attribute_writer);
	mutex_init(&kb_data->kb_connect_lock);

	ret = rm_hwmon_api_register_attributes(kb_data->parent_dev, ENDPOINT_KEYBOARD, kb_data,
					       attribute_configs, ARRAY_SIZE(attribute_configs));
	if (ret)
		goto remove_attributes;

	ret = rm_hwmon_api_register_endpoint(dev, ENDPOINT_KEYBOARD, rm_hwmon_keyboard_hsp_event);
	if (ret)
		goto remove_attributes;

	memset(&props, 0, sizeof(props));
	props.scale = BACKLIGHT_SCALE_LINEAR;
	props.type = BACKLIGHT_PLATFORM;
	props.brightness = kb_data->bl_brightness;
	props.max_brightness = MAX_BL_BRIGHTNESS;
	bd = devm_backlight_device_register(dev, "rm_keyboard_backlight",
					    kb_data->parent_dev, kb_data,
					    &rm_hwmon_kb_bl_ops, &props);

	ret = devm_device_add_groups(dev, dev_groups);
	if (ret) {
		dev_err(dev, "%s: Failed to add dev_groups\n", __func__);
		goto remove_attributes;
	}

	dev_info(dev, "RM HWMON keyboard module initialized\n");
	return 0;

remove_attributes:
	rm_hwmon_api_remove_attributes(kb_data->parent_dev, ENDPOINT_KEYBOARD,
				       attribute_configs, ARRAY_SIZE(attribute_configs));
	rm_hwmon_api_remove_endpoint(dev, ENDPOINT_KEYBOARD);
	return ret;
}

static int rm_hwmon_keyboard_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct kb_data *kb_data = dev_get_drvdata(dev);

	rm_hwmon_keyboard_unregister(kb_data);
	rm_hwmon_api_remove_attributes(kb_data->parent_dev, ENDPOINT_KEYBOARD,
				       attribute_configs, ARRAY_SIZE(attribute_configs));
	rm_hwmon_api_remove_endpoint(dev, ENDPOINT_KEYBOARD);
	return 0;
}

static int rm_hwmon_kb_led_save(struct device *dev)
{
	struct kb_data *pdata = dev_get_drvdata(dev);

	if (pdata->kb_dev && pdata->rm_key_light) {
		pdata->rm_key_on_after_resume = true;
		input_event(pdata->kb_dev, EV_LED, LED_MISC, 0);
	}

	return 0;
}

static int rm_hwmon_kb_led_load(struct device *dev)
{
	struct kb_data *pdata = dev_get_drvdata(dev);

	if (pdata->kb_dev && pdata->rm_key_on_after_resume)
		input_event(pdata->kb_dev, EV_LED, LED_MISC, 1);

	pdata->rm_key_on_after_resume = false;

	return 0;
}

static const struct dev_pm_ops rm_hwmon_kb_pm_ops = {
	.freeze = rm_hwmon_kb_led_save,
	.restore = rm_hwmon_kb_led_load,
	.thaw = rm_hwmon_kb_led_load,
	.suspend = rm_hwmon_kb_led_save,
	.resume = rm_hwmon_kb_led_load,
};

static struct platform_driver rm_hwmon_keyboard = {
	.probe = rm_hwmon_keyboard_probe,
	.remove = rm_hwmon_keyboard_remove,
	.driver	= {
		.name = "rm_hwmon_keyboard",
#ifdef CONFIG_PM
		.pm = &rm_hwmon_kb_pm_ops,
#endif
	},
};
module_platform_driver(rm_hwmon_keyboard);

MODULE_DESCRIPTION("reMarkable HWMON keyboard driver");
MODULE_AUTHOR("Kai André Venjum <kai.andre.venjum@remarkable.no>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rm_hwmon_keyboard");
