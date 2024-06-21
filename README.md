# map_area_filter
## Overview
Map area filter is devided into two filtering types, Pointcloud filter and Object filter.
### Pointcloud filter
This module erases all point clouds contained within the specified area.
### Object filter
This module erases all objects contained within the specified area. It is also possible to specify the type of object and erase only that object.
## Usage
### Setting Filtering Locations by csv file
Please create separate csv file for pointcloud filter and object filter respectively. You also need to give the path of csv file to parameter `map_area_csv`
#### Pointcloud filter
If you want to filter the edges and interior of a polygon whose vertex coordinates are (x1,y1),(x2,y2),...,(xn,yn), add the following lines to the csv
```
1,x1,y1,x2,y2,..., xn,yn
```
Please note that you have to write "1" at the beginning of the line.

#### Object filter
If you want to filter the edges and interior of a polygon whose vertex coordinates are (x1,y1),(x2,y2),...,(xn,yn) and delete objects of type `object_type`, add the following lines to the csv
```
object_type,x1,y1,x2,y2,...,xn,yn
```
`object_type`
|type_name|object_type|
|----|----|
|Unknown|0|
|Car|1|
|Truck|2|
|Bus|3|
|Trailer|4|
|Motorcycle|5|
|Bicycle|6|
|Pedestrian|7|
|delete all objects|8|

### Setting filter type
Please look at `map_area_filter.lauch.xml`. By default, both Pointcloud filter and Object filter nodes are activated. 
If you want to use only one filter, comment out the description of the filter node you do not want to use.

## memo
Topics of viewing filtering areas are `pointcloud_filter_area`, `objects_filter_area`.
