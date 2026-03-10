import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import pn532
from esphome.const import CONF_ID

AUTO_LOAD = ["pn532"]

desfire_ns = cg.esphome_ns.namespace("desfire_reader")
DesfireReader = desfire_ns.class_("DesfireReader", cg.Component)

CONF_PN532_ID = "pn532_id"
CONF_APPLICATION_ID = "application_id"
CONF_FILE_ID = "file_id"
CONF_KEY_ID = "key_id"
CONF_AUTHENTICATION_KEY = "authentication_key"
CONF_DATA_KEY = "data_key"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(DesfireReader),

        cv.Required(CONF_PN532_ID): cv.use_id(pn532.PN532),

        cv.Required(CONF_APPLICATION_ID): cv.hex_int,
        cv.Required(CONF_FILE_ID): cv.int_range(min=0, max=31),
        cv.Required(CONF_KEY_ID): cv.int_range(min=0, max=13),

        cv.Required(CONF_AUTHENTICATION_KEY): cv.string,
        cv.Optional(CONF_DATA_KEY): cv.string,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    pn = await cg.get_variable(config[CONF_PN532_ID])
    cg.add(var.set_pn532(pn))

    cg.add(var.set_application_id(config[CONF_APPLICATION_ID]))
    cg.add(var.set_file_id(config[CONF_FILE_ID]))
    cg.add(var.set_key_id(config[CONF_KEY_ID]))

    cg.add(var.set_authentication_key(config[CONF_AUTHENTICATION_KEY]))

    if CONF_DATA_KEY in config:
        cg.add(var.set_data_key(config[CONF_DATA_KEY]))