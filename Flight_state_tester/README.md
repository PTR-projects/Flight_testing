
## Installing
To install this software you must install CMake <br />
Go to source folder and run 
```shell
cmake ../Flight_state_tester
make
```

## Usage
To use simulation you need to configure your FSD file and add required macros to compile it in cpp.

## Results
Results will appear in .csv format as output.csv file in data folder. For data analysis you should use provided Python plotter script. To run it just type:
```shell
python3 Plotter.py
```
Be aware that plotter requires csv an matplotlib to run.

