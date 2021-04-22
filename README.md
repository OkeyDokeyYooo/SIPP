# CMPT 417 Final Proj

### Command to run generate_map.py

```bash
python generate_map.py --size 10 --num 1
```

- size: The Size of the map, will generate 10 by 10 map
- num: The number of maps will be generated

For instance: this command will create a folder **instances_size_10** on your current folder

and there will be one file text_1.txt under the new folder


### Command to run generate_instance.py

difference with map is generate instance will control one var, either number of agent or wall

```bash
python3 generate_instance.py --agent  --size 20
```

- agent: control the number of wall to be a fix number, and increase the number of agent for default 50 instance
- wall: same as agent
- size: same
- num: same


### Command to run run__experiments.py

The way to run CBS with SIPP and CBS with noraml A*

```bash
python run_experiments.py --instance "20_instances_size_agent/test_*" --solver CBS --batch --size 20 --agent
```

--instance: the file or folder to run the experiment

--solver: either CBS or CBS_N

--batch: run benchmark to generate CSV file

--size: the size of the map, using for generate the result file name

--agent: can not use this, having this means all the file, the only increase var is number of agents

--wall: same as wall
