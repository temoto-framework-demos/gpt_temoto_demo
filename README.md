# gpt_temoto_demo

This is a setup package for replicating the GPT + dual-arm robot demostration. 

It contains a set of TeMoto actions able to controla mobile manipulator to naviage, move the arms and take pictures. 

# Demo

The user can send an instruction for an inspection task, which has to navigate to and inspect specific areas defined by the operator through Natural Language commands and an Augmented Reality Interface. 

This package goes along with the [gpt parser node](https://github.com/temoto-framework/gpt_umrf_parser), and the [multi modality resolution](https://github.com/UTNuclearRobotics/multimodal_resolution_ar) that combines the voice instruction with interactive markers as strigs, and a server node appends some examples to generates a prompt request to OpenAI, which generates a UMRF graph. Once the response is received, it invokes the set of actions stored in this repository on the robot to perform the task.



