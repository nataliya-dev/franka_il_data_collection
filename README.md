# franka_il_data_collection
## Set up the workspace

```
uv venv venv --python 3.10
source venv/bin/activate
uv pip install -r requirements.txt
```

## Install franky
Make sure you have a real-time kernel installed and you allow the executing user to run real-time applications.
```
VERSION=0-9-2
wget https://github.com/TimSchneider42/franky/releases/latest/download/libfranka_${VERSION}_wheels.zip
unzip libfranka_${VERSION}_wheels.zip
uv pip install numpy
uv pip install --no-index --find-links=./dist franky-control
```

## Run the data collection
```
cd collect_data
python3 run_collection.py
```