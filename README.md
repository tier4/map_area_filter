# map_area_filter
## Overview
Map area filter is devided into two filtering types, Pointcloud filter and Object filter.
### Pointcloud filter
This module erases all point clouds contained within the specified area.
### Object filter
This module erases all objects contained within the specified area. It is also possible to specify the type of object and erase only that object.
## Usage
### Setting filter type (WIP)
Please look at `map_area_filter.lauch.xml`. By default, both Pointcloud filter and Object filter nodes are activated. 
If you want to use only one filter, comment out the description of the filter node you do not want to use.

### Setting filtering locations by lanelet
The filtering areas used by `map_area_filter` are defined by adding the following tags to **polygon** layers within the Lanelet2 map.

#### Attributes
These tags are required to define the area.

| Key | Value | Description |
| :--- | :--- | :--- |
| **type** | `map_area_filter` | Indicates that the area is to be loaded by map_area_filter. |
| **subtype** | `removal_area` | Indicates that the area is for object removal. |

#### Object specification
Specify the object types to be removed using optional tags.
To target multiple object types, add a separate tag for each type.
Note that objects will not be removed if their type is not registered as a Key or if the Value is anything other than `remove`.

* **Key**: Object type (see list below)
* **Value**: `remove`

**Supported Object Types (Key)**
* `all` (Targets all perception objects and point clouds)
* `all_objects` (Targets all perception objects)
* `pointcloud` (Targets point cloud data)
* `unknown`, `car`, `truck`, `bus`, `trailer`, `motorcycle`, `bicycle`, `pedestrian` (Targets the corresponding detection classes)

#### Height-based filtering (Optional Tags)
You can specify a height range to restrict the removal area.

| Key | Value (Ex.) | Description |
| :--- | :--- | :--- |
| **max_removal_height** | `0.5` | Removes objects located below this height |
| **min_removal_height** | `2.0` | Removes objects located above this height |

**Logic Notes**
* **Setting `max_removal_height: 0.5`**
  Used for removing noise near the ground (e.g., objects below 0.5m).
* **Setting `min_removal_height: 2.0`**
  Used to prevent false detections of overhead structures (e.g., objects above 2.0m).

## memo
Topics of viewing filtering areas are `pointcloud_filter_area`, `objects_filter_area`.
