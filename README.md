# Memory
Memory and Points of interest

Memory code heavily inspired and taken from fairo's droidlet's memory module 
`droidlet` provides a memory system that can store generic metadata. This memory system is used by the planner/controller to do tasks utilizing context provided by information stored in memory.

The memory is backed by an SQL database, and has schemas to represent common semantic information for robots and the environments

Added a BCIDetectedObjects table to base_memory_schema.sql
Added a BCIDetectedObjectFeatures table to loco_memory_schema.sql
Wrote note class for BCIDetectedObjectNode for OOI

# Yolov7
Please see yolov7.RE1_README.md to install and use

# RE1
Please see sample_demo.ipynb for for details on how to use

# Installation:
'''
git clone https://github.com/BatmanofZuhandArrgh/RE1.git
cd RE1
pip install -r requirements.txt

git submodule update --init
pip install -r yolov7/requirements.txt
'''