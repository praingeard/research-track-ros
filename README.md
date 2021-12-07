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
This node also implements the `/increase_speed` and `/reduce_speed` services.

#### Subscribed topics

- `/right_scan` robot right laserscan.
- `/left_scan` robot left laserscan.
- `/front_right_scan` robot front right laserscan.
- `/front_left_scan` robot front left laserscan.
- `/current_speed` current linear speed.

#### Published topics

- `/cmd_vel` desired velocities.

#### Services 

- `/increase_speed`(Empty) increase speed by 20%.
- `/reduce_speed`(Empty) reduce speed by 20%.

#### Algorithm

The node gets the difference in minimum distance between the front left and front right scans. Then it turns the robot away from the wall, with the current velocity as linear speed.
If the robot is too close to the front wall then the node gets the difference between right and left distances, slows down the robot and turns away from the closest wall.
The different services add or reduce the current speed by 20%.

### ui_node

This node implements the `/increase_speed`, `/reduce_speed` and `/reset_positions` service clients.

#### Algorithm

The node waits for a user input (between 1 and 3) and calls the corresponding service (add or reduce speed and reset).

## flowchart of the code

[![](https://mermaid.ink/img/eyJjb2RlIjoiZ3JhcGggVERcbiAgICBBW1NpbXVsYXRpb25dIC0tPnwvYmFzZV9zY2FufCBCW2FuZ2xlX2ZpbHRlcl9ub2RlXVxuICAgIEIgLS0-IEMoQ3V0IHRoZSBsYXNlcnNjYW4gbWVzc2FnZSBpbiBkaWZmZXJlbnQgcmFuZ2VzKVxuICAgIEMgLS0-fC9mcm9udF9sZWZ0X3NjYW58IERcbiAgICBDIC0tPnwvbGVmdF9zY2FufCBEW2NvbnRyb2xfbm9kZV1cbiAgICBDIC0tPnwvcmlnaHRfc2NhbnwgRFxuICAgIEMgLS0-fC9mcm9udF9yaWdodF9zY2FufCBEXG4gICAgRCAtLT4gfGlmIGZyb250IHdhbGwgaXMgdG9vIGNsb3NlfCBFKHNsb3cgZG93bilcbiAgICBFIC0tPiB8Z2V0IGRpc3RhbmNlIGJldHdlZW4gbGVmdCBhbmQgcmlnaHQgd2FsbHN8IElcbiAgICBEIC0tPiB8aWYgZnJvbnQgd2FsbCBpcyBmYXJ8IEcoZ2V0IHRoZSBkaWZmZXJlbmNlIGJldHdlZW4gZnJvbnQgbGVmdCBhbmQgZnJvbnQgcmlnaHQgZGlzdGFuY2VzKVxuICAgIEcgLS0-IEkoZHJpdmUgYXdheSBmcm9tIGNsb3Nlc3Qgd2FsbClcbiAgICBJIC0tPiB8L2NtZF92ZWx8IEgoY29udHJvbCByb2JvdClcbiAgICBVW3VzZXIgaW5wdXQgYmV0d2VlbiAxIGFuZCAzXSAtLT4gIEZbdWlfbm9kZV1cbiAgICBGIC0tPiBKKCByZWR1Y2Vfc3BlZWQgc2VydmljZSBjbGllbnQpXG4gICAgRiAtLT4gSyggaW5jcmVhc2Vfc3BlZWQgc2VydmljZSBjbGllbnQpXG4gICAgRiAtLT4gTCggcmVzZXRfcG9zaXRpb24gc2VydmljZSBjbGllbnQpXG4gICAgSyAtLT4gfC9pbmNyZWFzZV9zcGVlZCBzZXJ2aWNlIGNhbGx8RFxuICAgIEogLS0-IHwvcmVkdWNlX3NwZWVkIHNlcnZpY2UgY2FsbHxEXG4gICAgTCAtLT4gfC9yZXNldF9wb3NpdGlvbnMgc2VydmljZSBjYWxsfFpbc3RhZ2Vfcm9zIEFQSV1cblxuICAgICIsIm1lcm1haWQiOnsidGhlbWUiOiJkZWZhdWx0In0sInVwZGF0ZUVkaXRvciI6ZmFsc2UsImF1dG9TeW5jIjp0cnVlLCJ1cGRhdGVEaWFncmFtIjpmYWxzZX0)](https://mermaid-js.github.io/mermaid-live-editor/edit/#eyJjb2RlIjoiZ3JhcGggVERcbiAgICBBW1NpbXVsYXRpb25dIC0tPnwvYmFzZV9zY2FufCBCW2FuZ2xlX2ZpbHRlcl9ub2RlXVxuICAgIEIgLS0-IEMoQ3V0IHRoZSBsYXNlcnNjYW4gbWVzc2FnZSBpbiBkaWZmZXJlbnQgcmFuZ2VzKVxuICAgIEMgLS0-fC9mcm9udF9sZWZ0X3NjYW58IERcbiAgICBDIC0tPnwvbGVmdF9zY2FufCBEW2NvbnRyb2xfbm9kZV1cbiAgICBDIC0tPnwvcmlnaHRfc2NhbnwgRFxuICAgIEMgLS0-fC9mcm9udF9yaWdodF9zY2FufCBEXG4gICAgRCAtLT4gfGlmIGZyb250IHdhbGwgaXMgdG9vIGNsb3NlfCBFKHNsb3cgZG93bilcbiAgICBFIC0tPiB8Z2V0IGRpc3RhbmNlIGJldHdlZW4gbGVmdCBhbmQgcmlnaHQgd2FsbHN8IElcbiAgICBEIC0tPiB8aWYgZnJvbnQgd2FsbCBpcyBmYXJ8IEcoZ2V0IHRoZSBkaWZmZXJlbmNlIGJldHdlZW4gZnJvbnQgbGVmdCBhbmQgZnJvbnQgcmlnaHQgZGlzdGFuY2VzKVxuICAgIEcgLS0-IEkoZHJpdmUgYXdheSBmcm9tIGNsb3Nlc3Qgd2FsbClcbiAgICBJIC0tPiB8L2NtZF92ZWx8IEgoY29udHJvbCByb2JvdClcbiAgICBVW3VzZXIgaW5wdXQgYmV0d2VlbiAxIGFuZCAzXSAtLT4gIEZbdWlfbm9kZV1cbiAgICBGIC0tPiBKKCByZWR1Y2Vfc3BlZWQgc2VydmljZSBjbGllbnQpXG4gICAgRiAtLT4gSyggaW5jcmVhc2Vfc3BlZWQgc2VydmljZSBjbGllbnQpXG4gICAgRiAtLT4gTCggcmVzZXRfcG9zaXRpb24gc2VydmljZSBjbGllbnQpXG4gICAgSyAtLT4gfC9pbmNyZWFzZV9zcGVlZCBzZXJ2aWNlIGNhbGx8RFxuICAgIEogLS0-IHwvcmVkdWNlX3NwZWVkIHNlcnZpY2UgY2FsbHxEXG4gICAgTCAtLT4gfC9yZXNldF9wb3NpdGlvbnMgc2VydmljZSBjYWxsfFpbc3RhZ2Vfcm9zIEFQSV1cblxuICAgICIsIm1lcm1haWQiOiJ7XG4gIFwidGhlbWVcIjogXCJkZWZhdWx0XCJcbn0iLCJ1cGRhdGVFZGl0b3IiOmZhbHNlLCJhdXRvU3luYyI6dHJ1ZSwidXBkYXRlRGlhZ3JhbSI6ZmFsc2V9)