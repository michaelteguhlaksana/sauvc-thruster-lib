# include "thruster_aggregator.h"
# include "thruster.h"
# include <string>
# include <map>
# include <Arduino.h>
# include "config/config.h"
/***
 * The implementation of the function to setup the thruster aggregator
 * @return {bool} whether the setup was successful or not
 */
bool ThrusterAggregator::setup() {
    bool addition_success = this->add_thrusters({THRUSTER_ID_TO_ARDUINO_PIN});
    bool store_success = this->store_motion_to_thruster_id_and_esc_input_map();
    delay(1000);
    std::string setup_success {"false"};
    if (addition_success and store_success) {
        setup_success = "true";
    }
    else {
        setup_success = "false";
    }
    Serial.println("Thruster aggregator setup's success: " + String(setup_success.c_str()));
    return addition_success and store_success;
}

/**
 * The implementation of the function to add thrusters to thruster aggregator.
 * @param thrusters_id_and_pin_to_add {std::map<int, byte>} represents thrusters' id and pin to add
 * @return {bool} whether the addition was successful or not
 */
bool ThrusterAggregator::add_thrusters(const std::map<int, byte>& thrusters_id_and_pin_to_add) {
    for (const auto &[thruster_id, thruster_pin]: thrusters_id_and_pin_to_add) {
        thruster_id_to_instance_map.insert(std::pair<int, Thruster>(thruster_id, Thruster(thruster_id, thruster_pin)));
    }
    return thruster_id_to_instance_map.size() == thrusters_id_and_pin_to_add.size();
}

/**
 * The implementation of the function to store the mapping of motion type to run
 * @return {bool} indicates whether the storing was successful or not
 */
bool ThrusterAggregator::store_motion_to_thruster_id_and_esc_input_map() {
	Serial.println("ya");
    std::map<std::string, std::map<int, int>> loaded_predefined_motion =
            ThrusterAggregator::load_predefined_motion();
    Serial.println("yu");
    for ( const auto &[motion, motor_id_to_esc_input]: loaded_predefined_motion ) {
        motion_to_thruster_id_to_esc_input_map.insert(std::pair<std::string,
                std::map<int, int>>(motion, motor_id_to_esc_input));
    }
    return motion_to_thruster_id_to_esc_input_map.size() == loaded_predefined_motion.size();
}

/**
 * The implementation of the function to load pre-defined thrusters' motion.
 * @return {std::map<int,int>} the motion to thruster id to esc input map
 */
std::map<std::string, std::map<int, int>> ThrusterAggregator::load_predefined_motion() {
    return {
            {"forward", {THRUSTER_ID_TO_ESC_INPUT_FOR_FORWARD}},
            {"submerge", {THRUSTER_ID_TO_ESC_INPUT_FOR_SUBMERGE}},
            {"surface", {THRUSTER_ID_TO_ESC_INPUT_FOR_SURFACE}},
    };
}


/**
 * The implementation of the function to move thrusters according to a motion
 * @param motion {string} the motion for thrusters to produce
 * @return {bool} whether the execution of movement was successful or not
 */
bool ThrusterAggregator::move()
{
  if (this->id_to_esc_map != this->id_to_esc_map_prev)
  {
    this->id_to_esc_map_prev = this->id_to_esc_map;
    for (const auto &[id, esc]: this->id_to_esc_map)
    {
      this->thruster_id_to_instance_map[id].run(esc);
      this->thruster_id_to_actual_speed_map[id] = Thruster::get_safe_esc_input(esc);
    }
  }

  return true;
}

bool ThrusterAggregator::go(GoMotion motion)
{
  switch (motion)
  {
    case FORWARD:
    {
      //update esc map
      for (auto &[id, esc]: ID_TO_ESC_GO_FORWARD)
      {
        this->id_to_esc_map[id] = esc;
      }
      break;
    }
    case BACKWARD:
    {
      //update esc map
      for (auto &[id, esc]: ID_TO_ESC_GO_BACKWARD)
      {
        this->id_to_esc_map[id] = esc;
      }
      break;
    }
    case STOP:
    {
      //update esc map
      for (auto &[id, esc]: ID_TO_ESC_GO_STOP)
      {
        this->id_to_esc_map[id] = esc;
      }
      break;
    }

  }
}

bool ThrusterAggregator::vertical(VerticalMotion motion)
{
  switch (motion)
  {
    case SUBMERGE:
    {
      //update esc map
      for (auto &[id, esc]: ID_TO_ESC_VERTICAL_SUBMERGE)
      {
        this->id_to_esc_map[id] = esc;
      }
      break;
    }
    case SURFACE:
    {
      //update esc map
      for (auto &[id, esc]: ID_TO_ESC_VERTICAL_SURFACE)
      {
        this->id_to_esc_map[id] = esc;
      }
      break;
    }
    case STOP:
    {
      //update esc map
      for (auto &[id, esc]: ID_TO_ESC_VERTICAL_STOP)
      {
        this->id_to_esc_map[id] = esc;
      }
      break;
    }

  }
}

/**
 * The implementation of the function to stop all thrusters.
 * @return {bool} whether the stopping was successful or not
 */
bool ThrusterAggregator::stop() {
    for (const auto &[id, thruster]: thruster_id_to_instance_map) {
        thruster.stop();
        this->thruster_id_to_actual_speed_map[id] = int(ESC_INPUT_STOP);
    }
    return true;
}

/**
 * The implementation of the function to stabilise thrusters.
 * @return {bool} whether the stabilisation was successful or not
 */
bool ThrusterAggregator::stabilise() {
    for (const auto &[id, thruster]: this->thruster_id_to_instance_map) {
        if(this->thruster_id_to_stabilised_speed_map.find(id) == this->thruster_id_to_stabilised_speed_map.end()) {
            return false;
        }
    }
    for (const auto &[id, stabilised_esc_input]: this->thruster_id_to_stabilised_speed_map) {
        this->thruster_id_to_instance_map[id].run(stabilised_esc_input);
        this->thruster_id_to_actual_speed_map[id] = Thruster::get_safe_esc_input(stabilised_esc_input);
    }
    return true;
}

/**
 * A function to update thrusters' stabilised speed.
 * @param new_thruster_id_to_stabilised_speed_map {std::map <int, int>} thrusters' stabilised speed.
 */
void ThrusterAggregator::update_stabilised_speed(std::map<int, int> new_thruster_id_to_stabilised_speed_map) {
    for (const auto &[thruster_id, stabilised_speed]: new_thruster_id_to_stabilised_speed_map) {
        this->thruster_id_to_stabilised_speed_map[thruster_id] = new_thruster_id_to_stabilised_speed_map[thruster_id];
    }
}

/**
 * The implementation of the function to get the actual thrusters' speed.
 * @return {std::map<int, int>} actual thrusters' speed
 */
std::map <int, int> ThrusterAggregator::get_actual_thrusters_speed() {
    return this->thruster_id_to_actual_speed_map;
}
