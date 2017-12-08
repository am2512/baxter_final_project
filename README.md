# **Baxter as a Laundry Assistant:** opening and closing a Tide bottle
*Group 1: Ahalya, Wanning, Bill, Ola, Mike*


## Overview of project

This package (package name: me495_baxter_jar) allows Baxter to open and close a Tide bottle.

[photo of setup]

## How to run

A closed Tide bottle with an AR tag on the cap needs to be firmly attached (duct tape will suffice) to a table well within Baxter's reachable area. Once communication with Baxter is established run command:

`roslaunch me495_baxter_jar startup.launch`

If you would like to visualize Baxter and the tag in rviz to monitor what Baxter is seeing, run command:

`roslaunch me495_baxter_jar debug_ar_trakck.launch`

## State machine: nodes, services, and their functionality

The package operates similarly to a state machine. Upon completion of each task, the next one is called using a service.

Key steps:

1. Initiate Baxter:
  cameras, grippers, elaborate

1. Go to home position:
  blah bla blah

1. Locate AR tag:

1. Move to AR tag and prepare to grip:

1. Untwist wrist (CW) and unscrew lid (CCW):

1. Retract and move to drop cap on table:

1. Go to home position:

1. Locate AR tag:

1. Move to AR tag and prepare to grip:

1. Untwist wrist (CCW) and screw in lid (CW):

1. Retract and go to home position:


## Additional ROS packages required

1. **ar_track_alvar** - a ROS wrapper for Alvar, an open-source AR-tag-tracking library.

2.
