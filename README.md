# research-track-ros

This code implements a control node for a robot simulation. The speed of the robot can be increased or reduced using services. 

## use

Copy the 2 packages in your ros workspace src folder. 
Compile with `catkin_make`.
Run using the command `roslaunch robot_control robot_launch.xml`
To change speed you can call the `/reduce_speed` and `/increase_speed` empty services. 

## Nodes

### angle_filter

This node takes a range as parameters and returns the laserscan `/base_scan` filtered on this range on a chosen topic.

#### Subscribed topics

- `/base_scan` robot laserscan.

#### Published topics

- `/scan_topic` filtered laserscan.

#### Parameters 

- `min_angle(double)` minimum angle of the chosen range.
- `max_angle(double)` maximum angle of the chosen range.
- `scan_topic(string)` topic name for the published scan.

### control_node

This node publishes the desired twist of the robot on the `/cmd_vel` topic.

#### Subscribed topics

- `/right_scan` robot right laserscan.
- `/left_scan` robot left laserscan.
- `/front_right_scan` robot front right laserscan.
- `/front_left_scan` robot front left laserscan.
- `/current_speed` current linear speed..

#### Published topics

- `/cmd_vel` desired velocities.

#### Algorithm

The node gets the difference in minimum distance between the front left and front right scans. Then it turns the robot away from the wall, with the current velocity as linear speed.
If the robot is too close to the front wall then the node gets the difference between right and left distances, slows down the robot and turns away from the closest wall.

### ui_node

This node implements the `/increase_speed` and `/reduce_speed` services.

#### Published topics

- `/current_vel` current velocity.

#### Algorithm

Both services add or reduce the current speed by 20% and publish it on the `/current_vel` topic.

## flowchart of the code

[![](https://mermaid.ink/img/eyJjb2RlIjoiZ3JhcGggVERcbiAgICBBW1NpbXVsYXRpb25dIC0tPnwvYmFzZV9zY2FufCBCW2FuZ2xlX2ZpbHRlcl9ub2RlXVxuICAgIEIgLS0-IEMoQ3V0IHRoZSBsYXNlcnNjYW4gbWVzc2FnZSBpbiBkaWZmZXJlbnQgcmFuZ2VzKVxuICAgIEMgLS0-fC9mcm9udF9sZWZ0X3NjYW58IERcbiAgICBDIC0tPnwvbGVmdF9zY2FufCBEW2NvbnRyb2xfbm9kZV1cbiAgICBDIC0tPnwvcmlnaHRfc2NhbnwgRFxuICAgIEMgLS0-fC9mcm9udF9yaWdodF9zY2FufCBEXG4gICAgRCAtLT4gfGlmIGZyb250IHdhbGwgaXMgdG9vIGNsb3NlfCBFKHNsb3cgZG93bilcbiAgICBFIC0tPiB8Z2V0IGRpc3RhbmNlIGJldHdlZW4gbGVmdCBhbmQgcmlnaHQgd2FsbHN8IElcbiAgICBEIC0tPiB8aWYgZnJvbnQgd2FsbCBpcyBmYXJ8IEcoZ2V0IHRoZSBkaWZmZXJlbmNlIGJldHdlZW4gZnJvbnQgbGVmdCBhbmQgZnJvbnQgcmlnaHQgZGlzdGFuY2VzKVxuICAgIEcgLS0-IEkoZHJpdmUgYXdheSBmcm9tIGNsb3Nlc3Qgd2FsbClcbiAgICBJIC0tPiB8L2NtZF92ZWx8IEgoY29udHJvbCByb2JvdClcbiAgICBVW2luY3JlYXNlX3NwZWVkX3NlcnZpY2VdIC0tPiB8Y2FsbHwgRlt1aV9ub2RlXVxuICAgIFZbcmVkdWNlX3NwZWVkX3NlcnZpY2VdIC0tPiB8Y2FsbHwgRlxuICAgIEYgLS0-IEwoY2hhbmdlIHNwZWVkIGJ5IDIwJSBkZXBlbmRpbmcgb24gd2hpY2ggc2VydmljZSBpcyBjYWxsZWQpXG4gICAgTCAtLT4gfC9jdXJyZW50X3NwZWVkfERcbiAgICAiLCJtZXJtYWlkIjp7InRoZW1lIjoiZGVmYXVsdCJ9LCJ1cGRhdGVFZGl0b3IiOmZhbHNlLCJhdXRvU3luYyI6dHJ1ZSwidXBkYXRlRGlhZ3JhbSI6ZmFsc2V9)](https://mermaid-js.github.io/mermaid-live-editor/edit/#eyJjb2RlIjoiZ3JhcGggVERcbiAgICBBW1NpbXVsYXRpb25dIC0tPnwvYmFzZV9zY2FufCBCW2FuZ2xlX2ZpbHRlcl9ub2RlXVxuICAgIEIgLS0-IEMoQ3V0IHRoZSBsYXNlcnNjYW4gbWVzc2FnZSBpbiBkaWZmZXJlbnQgcmFuZ2VzKVxuICAgIEMgLS0-fC9mcm9udF9sZWZ0X3NjYW58IERcbiAgICBDIC0tPnwvbGVmdF9zY2FufCBEW2NvbnRyb2xfbm9kZV1cbiAgICBDIC0tPnwvcmlnaHRfc2NhbnwgRFxuICAgIEMgLS0-fC9mcm9udF9yaWdodF9zY2FufCBEXG4gICAgRCAtLT4gfGlmIGZyb250IHdhbGwgaXMgdG9vIGNsb3NlfCBFKHNsb3cgZG93bilcbiAgICBFIC0tPiB8Z2V0IGRpc3RhbmNlIGJldHdlZW4gbGVmdCBhbmQgcmlnaHQgd2FsbHN8IElcbiAgICBEIC0tPiB8aWYgZnJvbnQgd2FsbCBpcyBmYXJ8IEcoZ2V0IHRoZSBkaWZmZXJlbmNlIGJldHdlZW4gZnJvbnQgbGVmdCBhbmQgZnJvbnQgcmlnaHQgZGlzdGFuY2VzKVxuICAgIEcgLS0-IEkoZHJpdmUgYXdheSBmcm9tIGNsb3Nlc3Qgd2FsbClcbiAgICBJIC0tPiB8L2NtZF92ZWx8IEgoY29udHJvbCByb2JvdClcbiAgICBVW2luY3JlYXNlX3NwZWVkX3NlcnZpY2VdIC0tPiB8Y2FsbHwgRlt1aV9ub2RlXVxuICAgIFZbcmVkdWNlX3NwZWVkX3NlcnZpY2VdIC0tPiB8Y2FsbHwgRlxuICAgIEYgLS0-IEwoY2hhbmdlIHNwZWVkIGJ5IDIwJSBkZXBlbmRpbmcgb24gd2hpY2ggc2VydmljZSBpcyBjYWxsZWQpXG4gICAgTCAtLT4gfC9jdXJyZW50X3NwZWVkfERcbiAgICAiLCJtZXJtYWlkIjoie1xuICBcInRoZW1lXCI6IFwiZGVmYXVsdFwiXG59IiwidXBkYXRlRWRpdG9yIjpmYWxzZSwiYXV0b1N5bmMiOnRydWUsInVwZGF0ZURpYWdyYW0iOmZhbHNlfQ)