# Preprocessing helper scripts


## SWARCO Mastra file format parser -- mastraparse.py

### Requirements

  * Python 3.6
  * Google OR-Tools package
  * Numpy

#### Python 3.6
[Download Python](https://www.python.org/downloads/)

#### Google OR-Tools
[Website](https://developers.google.com/optimization/)

[Official installation instructions](https://developers.google.com/optimization/install/python/)

Note: If needed, replace `python` with `python3` or `python36` (check with `python --version`)
```shell
python -m pip install --upgrade pip
python -m pip install ortools
```

#### Numpy
```shell
python -m pip install numpy
```

### Using it

You can either import the file as a local module, or modify the `main` function directly.
#### To parse the data: 
```python
data = mastra(path_to_mastra_file)
```
It is important that you give the path, and not a file, as the parser automatically opens the file at given path.


#### To get the readings:
```python
data.get_periods()
```
This will return a dictionary where the keys are date/time tuples of the form `('yymmdd','hhmm')`, and the values are lists of readings where index 0 is the first detector, index 1 is the second etc...

#### To aggregate the readings further:
```python
aggregated = data.aggregate_readings(interval)
```
This returns a list of `numpy` arrays containing vehicle count data, in order of time, but the actual time is lost.

The parameter `interval` determines how many minutes of data should be aggregated, and should be a multiple of the original data interval (5 minutes).

As an example, `data.aggregate_readings(5)` will return a list containing every original reading (As the original file report vehicle counts every 5 minutes), but in the form:
```python
[
    ...
    array([ 6,  6, 10,  0, 10,  0,  4,  4,  3,  1,  1,  1,  0,  1,  0,  0,  0,
            0,  1,  5,  3,  1,  2,  5,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
            0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0]),
    ...
]
```

#### To get number of vehicles passing some ordering of induction loops:

Firstly, some information on the possible detector orderings is needed.
The format used is a list of int tuples, where each tuple represents an order of induction loops a vehicle can pass.

In the below code, `(1, 2)` indicates that a vehicle could pass just detector 1 and 2, and then no other detectors in the network.
```python
detector_routes = [('1', '2'),
                   ('1', '2', '3', '23', '22'),
                   ('1', '2', '5', '24'),
                   ('1', '2', '5', '24', '20'),
                   ('1', '4', '3', '23', '22'),
                   ('3', '23', '22'),
                   ('5', '24'),
                   ('5', '24', '20'),
                   ('7', '8', '9', '21'),
                   ('7', '8', '10', '11', '22'),
                   ('7', '8', '12'),
                   ('14', '15'),
                   ('14', '15', '20'),
                   ('14', '19'),
                   ('16', '20'),
                   ('16', '21'),
                   ('16', '22')]
```
The above was generated programatically, and is in string form. To make things a bit easier, the function `str_tuple_list_to_int_tuple_list` was created, and simply converts the strings to the integers needed by `get_route_vehicle_counts`.

```python
get_route_vehicle_counts(self, routes, granularity=5, oidx=False)
```
Where `routes` is the data structure mentioned above, `granularity` is an aggregation parameter, and it is recommended to keep this equal to the input data frequency, or at least a multiple thereof.
If the induction loop id's are one indexed, `oidx` should be set to true.

The result is a list of dictionaries of the form:
```python
[
    {(1, 2): 48,
     (3, 23, 22): 9,
     (5, 24, 20): 38,
     (7, 8, 9, 21): 24,
     (7, 8, 10, 11, 22): 7,
     (7, 8, 12): 7,
     (14, 15, 20): 5,
     (14, 19): 3,
     (16, 20): 3},
     ...
]
```
Each dictionary in the list gives the counts for an interval of `granularity` minutes.
In the above case, the detector ordering `(1, 2)` had `48` vehicles pass in the first 5-minute period.