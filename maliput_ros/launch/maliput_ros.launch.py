# BSD 3-Clause License
#
# Copyright (c) 2022, Woven Planet.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
import launch
import launch_ros
import lifecycle_msgs

def generate_launch_description():
    # YAML configuration path to load the maliput plugin configuration.
    maliput_yaml_path = launch.substitutions.LaunchConfiguration('maliput_yaml_path')

    # YAML configuration path argument, to let the user configure it.
    # The 'maliput_yaml_path' variable will hold this value.
    maliput_yaml_path_arg = launch.actions.DeclareLaunchArgument(
        'maliput_yaml_path',
        default_value='',
        description='Path to the YAML file containing the maliput plugin configuration.')
    
    # The query server node.
    maliput_query_server_node = launch_ros.actions.LifecycleNode(
        package='maliput_ros', executable='maliput_ros', name='query_server', output='screen',
        parameters=[{"yaml_configuration_path": maliput_yaml_path}])
    
    # When the query_server reaches the 'inactive' state, make it take the 'activate' transition.
    register_event_handler_for_query_server_reaches_inactive_state = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=maliput_query_server_node, goal_state='inactive',
            entities=[
                launch.actions.LogInfo(
                    msg="node 'query_server' reached the 'inactive' state, 'activating'."),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(maliput_query_server_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    # When the query_server node reaches the 'active' state, log a message.
    register_event_handler_for_query_server_reaches_active_state = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=maliput_query_server_node, goal_state='active',
            entities=[
                launch.actions.LogInfo(
                    msg="node 'query_server' reached the 'active' state, launching 'listener'."),
            ],
        )
    )

    # Make the talker node take the 'configure' transition.
    emit_event_to_request_that_query_server_does_configure_transition = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(maliput_query_server_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    return launch.LaunchDescription([
        maliput_yaml_path_arg,
        register_event_handler_for_query_server_reaches_inactive_state,
        register_event_handler_for_query_server_reaches_active_state,
        maliput_query_server_node,
        emit_event_to_request_that_query_server_does_configure_transition,
    ])
