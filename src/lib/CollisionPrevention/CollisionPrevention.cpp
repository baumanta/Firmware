/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file CollisionPrevention.cpp
 * CollisionPrevention controller.
 *
 */

#include <CollisionPrevention/CollisionPrevention.hpp>
using namespace matrix;
using namespace time_literals;


CollisionPrevention::CollisionPrevention(ModuleParams *parent) :
	ModuleParams(parent)
{

}

CollisionPrevention::~CollisionPrevention()
{
	//unadvertise publishers
	if (_mavlink_log_pub != nullptr) {
		orb_unadvertise(_mavlink_log_pub);
	}
}

bool CollisionPrevention::initializeSubscriptions(SubscriptionArray &subscription_array)
{
	if (!subscription_array.get(ORB_ID(obstacle_distance), _sub_obstacle_distance)) {
		return false;
	}

	return true;
}

void CollisionPrevention::publishConstrainedSetpoint(const Vector2f &original_setpoint,
		const Vector2f &adapted_setpoint)
{

	collision_constraints_s	constraints;	/**< collision constraints message */

	//fill in values
	constraints.timestamp = hrt_absolute_time();

	constraints.original_setpoint[0] = original_setpoint(0);
	constraints.original_setpoint[1] = original_setpoint(1);
	constraints.adapted_setpoint[0] = adapted_setpoint(0);
	constraints.adapted_setpoint[1] = adapted_setpoint(1);

	// publish constraints
	if (_constraints_pub != nullptr) {
		orb_publish(ORB_ID(collision_constraints), _constraints_pub, &constraints);

	} else {
		_constraints_pub = orb_advertise(ORB_ID(collision_constraints), &constraints);
	}
}

void CollisionPrevention::calculateConstrainedSetpoint(Vector2f &setpoint, const Vector2f &position,
		const Vector2f &velocity)
{
	const obstacle_distance_s &obstacle_distance = _sub_obstacle_distance->get();
	Vector2f setpoint_dir = setpoint.normalized();

	if (hrt_elapsed_time(&obstacle_distance.timestamp) < RANGE_STREAM_TIMEOUT_US) {
		if (setpoint.norm() > 0.001f) {
			int distances_array_size = sizeof(obstacle_distance.distances) / sizeof(obstacle_distance.distances[0]);

			for (int i = 0; i < distances_array_size; i++) {
				//determine if distance bin is valid and contains a valid distance measurement
				if (obstacle_distance.distances[i] < obstacle_distance.max_distance &&
				    obstacle_distance.distances[i] > obstacle_distance.min_distance && i * obstacle_distance.increment < 360
				    && setpoint.norm() > 0.001f) {
					float distance = obstacle_distance.distances[i] / 100.0f; //convert to meters
					float angle = math::radians((float)i * obstacle_distance.increment);

					//calculate setpoint and velocity components in current bin direction
					Vector2f bin_direction = {cos(angle), sin(angle)};
					float sp_parallel = setpoint.dot(bin_direction);
					float vel_parallel = velocity.dot(bin_direction);

					//calculate max allowed velocity with a P-controller (same gain as in the position controller)
					float delay_distance = vel_parallel * _param_mpc_col_prev_dly.get();
					float vel_max = math::max(0.f, _param_mpc_xy_p.get() * (distance - _param_mpc_col_prev_d.get() - delay_distance));

					//limit the setpoint if the commanded velocity exceeds the calculated maximum
					if (sp_parallel > vel_max) {
						float new_vel = vel_max / bin_direction.dot(setpoint_dir);
						setpoint = new_vel * setpoint_dir;
					}
				}
			}
		}

	} else if (_last_message + MESSAGE_THROTTLE_US < hrt_absolute_time()) {
		mavlink_log_critical(&_mavlink_log_pub, "No range data received");
		_last_message = hrt_absolute_time();
	}
}

void CollisionPrevention::modifySetpoint(Vector2f &original_setpoint, const float max_speed, const Vector2f &position,
		const Vector2f &velocity)
{
	//calculate movement constraints based on range data
	Vector2f new_setpoint = original_setpoint;
	calculateConstrainedSetpoint(new_setpoint, position, velocity);

	//warn user if collision prevention starts to interfere
	bool currently_interfering = (new_setpoint(0) < original_setpoint(0) - 0.05f * max_speed
				      || new_setpoint(0) > original_setpoint(0) + 0.05f * max_speed
				      || new_setpoint(1) < original_setpoint(1) - 0.05f * max_speed
				      || new_setpoint(1) > original_setpoint(1) + 0.05f * max_speed);

	if (currently_interfering && (currently_interfering != _interfering)) {
		mavlink_log_critical(&_mavlink_log_pub, "Collision Warning");
	}

	_interfering = currently_interfering;
	publishConstrainedSetpoint(original_setpoint, new_setpoint);
	original_setpoint = new_setpoint;
}
