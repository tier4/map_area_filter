# map_area_filter
## Overview
Map area filter is devided into two filtering types, Pointcloud filter and Object filter.
### Pointcloud filter
This module erases all point clouds contained within the specified area.
### Object filter
This module erases all objects contained within the specified area. It is also possible to specify the type of object and erase only that object.

### Height-based filtering (Optional)
* If tags are not set: No height checks are performed, i.e., only the horizontal checks are performed.
* If height-related tags are set: Objects are removed only if they are located in the region above min_removal_height **OR** in the region below max_removal_height.

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
This feature operates based on the centroid height of the Lanelet polygon. Therefore, please ensure that appropriate elevation values are set for the polygon's vertices. Generally, it is recommended to align the height with the roadway (not the sidewalk).
Unless specific tags for the height filter function are configured, the height values of the polygon do not affect functionality.

| Key | Value (Ex.) | Description |
| :--- | :--- | :--- |
| **max_removal_height** | `0.5` | Removes objects located below this height |
| **min_removal_height** | `2.0` | Removes objects located above this height |

**Logic Notes**
* **Setting `max_removal_height: 0.5`**
  Used for removing noise near the ground (e.g., objects below 0.5m).
* **Setting `min_removal_height: 2.0`**
  Used to prevent false detections of overhead structures (e.g., objects above 2.0m).

#### Example
```xml
  <way id="89151">
    <nd ref="89141"/>
    <nd ref="89145"/>
    <nd ref="89146"/>
    <nd ref="89150"/>
    <nd ref="89141"/>
    <tag k="type" v="map_area_filter"/>
    <tag k="subtype" v="removal_area"/>
    <tag k="area" v="yes"/>
    <tag k="unknown" v="remove"/>
    <tag k="pointcloud" v="remove"/>
  </way>
```

## memo
Topics of viewing filtering areas are `pointcloud_filter_area`, `objects_filter_area`.
