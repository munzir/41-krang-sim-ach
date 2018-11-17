/*
 * Copyright (c) 2018, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @file motor.cpp
 * @author Munzir Zafar
 * @date Nov 15, 2018
 * @brief Handles ach communication for a specific motor group
 */

//============================================================================
MotorGroup::MotorGroup(
    const somatic_d_t& daemon, const char* cmd_chan_name,
    const char* state_chan_name, const std::vector<int>& joint_indices,
    const std::vector<double>& sign = std::vector<double>()) {
  daemon_ = &daemon;
  n_ = joint_indices.size();

  // Open ach channels for pub/sub
  somatic_d_channel_open(daemon_, &cmd_chan_, cmd_chan_name, NULL);
  somatic_d_channel_open(daemon_, &state_chan_, state_chan_name, NULL);
  ach_flush(&cmd_chan_);

  // The indices of the robot whose data we want to communicate
  joint_indices_ = joint_indices;

  // the sign to multiply dart data with before sending it off to ach
  sign_ = (!sign.empty()? sign : std::vector<double>(n_, 1.0);

  // Assign default values to message
  InitMessage();
}

//============================================================================
/// Sets up the message we will be sending to the motor group
void MotorGroup::InitMessage() {
  // Initialize the message and set status
  somatic__motor_state__init(&state_msg_);
  state_msg_.has_status = 1;
  state_msg_.status = SOMATIC__MOTOR_STATUS__MOTOR_OK;

  // Set the addresses (?)
  state_msg_.position = &state_msg_fields_.position_;
  state_msg_.velocity = &state_msg_fields_.velocity_;
  state_msg_.current = &state_msg_fields_.current_;

  // Initialize each of the field vectors
  somatic__vector__init(state_msg_.position);
  somatic__vector__init(state_msg_.velocity);
  somatic__vector__init(state_msg_.current);

  // Set the number of variables
  state_msg_.position->n_data = cx->n;
  state_msg_.velocity->n_data = cx->n;
  state_msg_.current->n_data = cx->n;

  // Set the message type
  state_msg_.meta = somatic_metadata_alloc();
  state_msg_.meta->type = SOMATIC__MSG_TYPE__MOTOR_STATE;
  state_msg_.meta->has_type = 1;
}
//========================================================================
//// Send state on ach channel
MotorGroup::SendState(const std::vector<double>& all_pos,
                      const std::vector<double>& all_vel,
                      const std::vector<double>& all_cur) {
  // Read values of our group specified by joint_indices_ in the state_msg_
  static double pos_vals[n_], vel_vals[n_], cur_vals[n_];
  for (int i = 0; i < n_; i++) {
    pos_vals[i] = sign_[i] * all_pos[joint_indices_[i]];
    vel_vals[i] = sign_[i] * all_vel[joint_indices_[i]];
    cur_vals[i] = sign_[i] * all_cur[joint_indices_[i]];
  }
  state_msg_.position->data = pos_vals;
  state_msg_.velocity->data = vel_vals;
  state_msg_.current->data = cur_vals;

  // Status message (always good)
  state_msg_.has_status = 1;
  state_msg_.status = SOMATIC__MOTOR_STATUS__MOTOR_OK;

  // Package a state message for the ack returned, and send to state channel
  r = SOMATIC_PACK_SEND(&state_chan_, somatic__motor_state, state_msg_);

  /// check message transmission
  somatic_d_check(daemon_, SOMATIC__EVENT__PRIORITIES__CRIT,
                  SOMATIC__EVENT__CODES__COMM_FAILED_TRANSPORT, ACH_OK == r,
                  "update_state", "ach result: %s", ach_result_to_string(r));
}

