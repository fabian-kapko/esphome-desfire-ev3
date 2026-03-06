import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, text_sensor, binary_sensor
from esphome.const import CONF_ID

AUTO_LOAD = ["text_sensor", "binary_sensor"]

desfire_ns = cg.esphome_ns.namespace("desfire_reader")
DesfireReaderComponent = desfire_ns.class_(
    "DesfireReaderComponent",
    cg.PollingComponent,
    i2c.I2CDevice,
)

CONF_APP_ID   = "app_id"
CONF_APP_KEY  = "app_key"   # AES-128 key for app authentication
CONF_DATA_KEY = "data_key"  # AES-128 key to decrypt file contents
CONF_RESULT   = "result"
CONF_AUTH_OK  = "auth_ok"
CONF_UID      = "uid"


def validate_hex_bytes(expected_len, label):
    def _validate(value):
        parts = str(value).split(":")
        if len(parts) != expected_len:
            raise cv.Invalid(f"{label} must be {expected_len} bytes (XX:XX format)")
        try:
            return [int(p, 16) for p in parts]
        except ValueError:
            raise cv.Invalid(f"{label}: invalid hex '{value}'")
    return _validate


CONFIG_SCHEMA = (
    cv.Schema({
        cv.GenerateID(): cv.declare_id(DesfireReaderComponent),
        cv.Required(CONF_APP_ID):   validate_hex_bytes(3,  "app_id"),
        cv.Required(CONF_APP_KEY):  validate_hex_bytes(16, "app_key"),
        cv.Required(CONF_DATA_KEY): validate_hex_bytes(16, "data_key"),
        cv.Optional(CONF_RESULT):   text_sensor.text_sensor_schema(),
        cv.Optional(CONF_AUTH_OK):  binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_UID):      text_sensor.text_sensor_schema(),
    })
    .extend(i2c.i2c_device_schema(0x24))
    .extend(cv.polling_component_schema("500ms"))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    aid = config[CONF_APP_ID]
    cg.add(var.set_app_id(aid[0], aid[1], aid[2]))

    cg.add(var.set_app_key(config[CONF_APP_KEY]))
    cg.add(var.set_data_key(config[CONF_DATA_KEY]))

    if CONF_RESULT in config:
        sens = await text_sensor.new_text_sensor(config[CONF_RESULT])
        cg.add(var.set_result_sensor(sens))

    if CONF_AUTH_OK in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_AUTH_OK])
        cg.add(var.set_auth_sensor(sens))

    if CONF_UID in config:
        sens = await text_sensor.new_text_sensor(config[CONF_UID])
        cg.add(var.set_uid_sensor(sens))
