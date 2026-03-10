import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor, binary_sensor, pn532_i2c
from esphome.const import CONF_ID

desfire_ns = cg.esphome_ns.namespace("desfire_reader")
DesfireReader = desfire_ns.class_("DesfireReader", cg.Component)

CONF_APP_ID = "app_id"
CONF_APP_KEY = "app_key"
CONF_FILE_ID = "file_id"
CONF_FILE_OFFSET = "file_offset"
CONF_MAX_PAYLOAD = "max_payload_len"

CONFIG_SCHEMA = cv.Schema(
{
cv.GenerateID(): cv.declare_id(DesfireReader),

cv.Required(CONF_APP_ID): cv.hex_uint32_t,
cv.Required(CONF_APP_KEY): cv.hex_bytes,

cv.Optional(CONF_FILE_ID, default=1): cv.uint8_t,
cv.Optional(CONF_FILE_OFFSET, default=0): cv.uint32_t,
cv.Optional(CONF_MAX_PAYLOAD, default=32): cv.uint16_t,

cv.Optional("result"): text_sensor.text_sensor_schema(),
cv.Optional("uid"): text_sensor.text_sensor_schema(),
cv.Optional("auth_ok"): binary_sensor.binary_sensor_schema(),

}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_app_id(config[CONF_APP_ID]))
    cg.add(var.set_app_key(config[CONF_APP_KEY]))

    cg.add(var.set_file_id(config[CONF_FILE_ID]))
    cg.add(var.set_file_offset(config[CONF_FILE_OFFSET]))
    cg.add(var.set_max_payload_len(config[CONF_MAX_PAYLOAD]))

    if "result" in config:
        sens = await text_sensor.new_text_sensor(config["result"])
        cg.add(var.set_result_sensor(sens))

    if "uid" in config:
        sens = await text_sensor.new_text_sensor(config["uid"])
        cg.add(var.set_uid_sensor(sens))

    if "auth_ok" in config:
        sens = await binary_sensor.new_binary_sensor(config["auth_ok"])
        cg.add(var.set_auth_sensor(sens))