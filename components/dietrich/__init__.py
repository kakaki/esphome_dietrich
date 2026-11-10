import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, uart
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_CELSIUS,
    UNIT_HOUR,
    UNIT_PERCENT,
    UNIT_REVOLUTIONS_PER_MINUTE,
)

CODEOWNERS = ["@kakaki"]
AUTHORS = ["@kakaki", "Claude Fable 5 (Anthropic)"]
DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor"]
MULTI_CONF = False

dietrich_ns = cg.esphome_ns.namespace("dietrich")
Dietrich = dietrich_ns.class_("Dietrich", cg.PollingComponent, uart.UARTDevice)

CONF_VARIANT = "variant"
DietrichVariant = dietrich_ns.enum("DietrichVariant")
VARIANTS = {
    "mcr3": DietrichVariant.DIETRICH_VARIANT_MCR3,
    "calenta_v1_p5": DietrichVariant.DIETRICH_VARIANT_CALENTA_V1_P5,
}


def _temp_schema():
    return sensor.sensor_schema(
        unit_of_measurement=UNIT_CELSIUS,
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
    )


def _percent_schema():
    return sensor.sensor_schema(
        unit_of_measurement=UNIT_PERCENT,
        accuracy_decimals=0,
        state_class=STATE_CLASS_MEASUREMENT,
    )


def _rpm_schema():
    return sensor.sensor_schema(
        unit_of_measurement=UNIT_REVOLUTIONS_PER_MINUTE,
        accuracy_decimals=0,
        state_class=STATE_CLASS_MEASUREMENT,
    )


def _hours_schema():
    return sensor.sensor_schema(
        unit_of_measurement=UNIT_HOUR,
        accuracy_decimals=0,
        state_class=STATE_CLASS_TOTAL_INCREASING,
    )


def _count_schema():
    return sensor.sensor_schema(
        accuracy_decimals=0,
        state_class=STATE_CLASS_TOTAL_INCREASING,
    )


def _bit_schema():
    return sensor.sensor_schema(accuracy_decimals=0)


def _raw_schema():
    return sensor.sensor_schema(accuracy_decimals=0)


SENSOR_SCHEMAS = {
    # frame status/state
    "state": _raw_schema(),
    "sub_state": _raw_schema(),
    "lockout": _raw_schema(),
    "blocking": _raw_schema(),
    # sample data - temperatures
    "flow_temp": _temp_schema(),
    "return_temp": _temp_schema(),
    "dhw_in_temp": _temp_schema(),
    "outside_temp": _temp_schema(),
    "calorifier_temp": _temp_schema(),
    "boiler_control_temp": _temp_schema(),
    "room_temp": _temp_schema(),
    "ch_setpoint": _temp_schema(),
    "dhw_setpoint": _temp_schema(),
    "room_temp_setpoint": _temp_schema(),
    # fan
    "fan_speed_setpoint": _rpm_schema(),
    "fan_speed": _rpm_schema(),
    # power / misc
    "ionisation_current": sensor.sensor_schema(
        unit_of_measurement="µA",
        accuracy_decimals=1,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    "internal_setpoint": _percent_schema(),
    "available_power": _percent_schema(),
    "pump_percentage": _percent_schema(),
    "desired_max_power": _percent_schema(),
    "actual_power": _percent_schema(),
    # demand source bits
    "demand_source_bit0": _bit_schema(),  # Mod.Controller Connected
    "demand_source_bit1": _bit_schema(),  # Heat demand from Mod.Controller
    "demand_source_bit2": _bit_schema(),  # Heat demand from on/off controller
    "demand_source_bit3": _bit_schema(),  # Frost Protection
    "demand_source_bit4": _bit_schema(),  # DHW Eco
    "demand_source_bit5": _bit_schema(),  # DHW Blocking
    "demand_source_bit6": _bit_schema(),  # Anti Legionella
    "demand_source_bit7": _bit_schema(),  # DHW Heat Demand
    # input bits
    "input_bit0": _bit_schema(),  # Shutdown Input
    "input_bit1": _bit_schema(),  # Release Input
    "input_bit2": _bit_schema(),  # Ionisation
    "input_bit3": _bit_schema(),  # Flow Switch detecting DHW
    "input_bit5": _bit_schema(),  # Min Gas Pressure
    "input_bit6": _bit_schema(),  # CH Enable
    "input_bit7": _bit_schema(),  # DHW Enable
    # valve bits
    "valve_bit0": _bit_schema(),  # Gas Valve
    "valve_bit2": _bit_schema(),  # Ignition
    "valve_bit3": _bit_schema(),  # 3-Way valve position
    "valve_bit4": _bit_schema(),  # Ext. 3-Way Valve
    "valve_bit6": _bit_schema(),  # Ext. Gas Valve
    # pump bits
    "pump_bit0": _bit_schema(),  # Pump
    "pump_bit1": _bit_schema(),  # Calorifier Pump
    "pump_bit2": _bit_schema(),  # Ext. CH Pump
    "pump_bit4": _bit_schema(),  # Status Report
    "pump_bit7": _bit_schema(),  # Opentherm SmartPower
    # misc sample data
    "hydro_pressure": sensor.sensor_schema(
        unit_of_measurement="bar",
        accuracy_decimals=2,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    "hru": _bit_schema(),
    "control_temp": _temp_schema(),
    "dhw_flowrate": sensor.sensor_schema(
        unit_of_measurement="L/min",  # Home Assistant's spelling
        accuracy_decimals=0,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    # counter data 1
    "hours_run_pump": _hours_schema(),
    "hours_run_3way": _hours_schema(),
    "hours_run_ch": _hours_schema(),
    "hours_run_dhw": _hours_schema(),
    "power_supply_aval_hours": _hours_schema(),
    "pump_starts": _count_schema(),
    "number_of_3way_valve_cycles": _count_schema(),
    "burner_start_dhw": _count_schema(),
    # counter data 2
    "total_burner_start": _count_schema(),
    "failed_burner_start": _count_schema(),
    "number_flame_loss": _count_schema(),
}

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Dietrich),
            cv.Optional(CONF_VARIANT, default="mcr3"): cv.enum(VARIANTS, lower=True),
            **{cv.Optional(key): schema for key, schema in SENSOR_SCHEMAS.items()},
        }
    )
    .extend(cv.polling_component_schema("15s"))
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    cg.add(var.set_variant(config[CONF_VARIANT]))

    for key in SENSOR_SCHEMAS:
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(var, f"set_{key}_sensor")(sens))
