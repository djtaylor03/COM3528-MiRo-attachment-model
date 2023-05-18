# COM3528-MiRo-attachment-model
Creating a human-MiRo attachment model for COM3528 Cognitive and Biomimetic Robotics.
This will project focus on giving MiRo's a type of 'emotional' connection to a human caregiver. This entails a MiRo behaving and exploring in different ways while still maintaing a 'relationship of care' from a staionary human (as seen in the 'Strange Situation' as developed by Mary Ainsworth)

Developers:
Leon Joyce,
Antrea Koulouteri,
Lisa Smith,
Daniel Taylor


Launch Instructions
The launch commands for each default attachment type
secure:
roslaunch miro_attachment full.launch

avoidant:
roslaunch miro_attachment full.launch attachment:=av

ambivalent:
roslaunch miro_attachment full.launch attachment:=am

Parameters am and av can be edited in src/attachment.py if custom values are desired
