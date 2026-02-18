# obstacle_removal_area_filter
This repository (https://github.com/tier4/map_area_filter) will be deprecated.
The map_area_filter package has been renamed to obstacle_removal_area_filter and migrated to the following location:
https://github.com/tier4/l4_toolkit/tree/main/obstacle_removal_area_filter

## Overview
The `obstacle_removal_area_filter` is a feature that removes input data—specifically "Objects" or "PointClouds"—that are located within specified areas and outputs the remaining data.

While not mandatory, this feature aims to enhance the stability and value of the autonomous driving system by incorporating human judgment for assistance and adjustment. It is designed to be placed and used **between the Perception and Planning modules of Autoware**.

![pipeline](./docs/pipeline_architecture.png)

Removal areas are defined on the High-Definition (HD) map (Lanelet2). By setting Lanelet tags, it is possible to remove only specific object types (pedestrians, vehicles, point clouds, etc.) or limit the removal scope using height restrictions.

## Configuration and Behavior

### Parameter Settings (Global)
You can globally configure whether to apply the filter to Objects and PointClouds respectively.
While it is possible to configure settings per area, please adjust these global settings if you wish to skip the processing entirely, such as to reduce computational load.

### Area Configuration (Lanelet2)
Each Lanelet polygon used as a removal area requires the following settings:

1.  **Type Designation**: A tag indicating that it is a target for `obstacle_removal_area_filter`.
2.  **Area Shape**: A polygon created from a sequence of points on the Lanelet.
3.  **Removal Target**: The type of object to remove (specified by tag).
4.  **Height Limit**: Specification of height region (optional).

![vmb](./docs/vmb_config.png)

#### 1. Lanelet Type
Define a `lanelet` element in the **Polygon** layer and set the `type` tag to `obstacle_removal_area`.
The `subtype` is not required.

#### 2. Area Shape
Configure the polygon using a sequence of points on the Lanelet. Please set an appropriate shape considering the inclusion determination logic.
While the inclusion check (inside/outside) uses only xy (horizontal) coordinates, appropriate settings for z-coordinates are also required for visualization purposes.

**Polygon vs. Object Inclusion Logic**:
  * **Objects**: If the **centroid** of the object is within the area, that object is removed (not output).
  * **PointCloud**: If a point is within the area, that point is removed (not output).

> **Note**: The judgment for objects is based strictly on the "centroid." It does not check if the "object's bounding box overlaps with the area" or if the "bounding box is completely contained." (*Note: If necessary, implementing an option to switch this logic is relatively easy.*)

#### 3. Object Type Configuration
Specify the object types you want to remove using tags. To target multiple types, add a tag for each one.
If an object type is not registered as a Key, or if the Value is not `remove`, it will not be removed.

* **Key**: Object Type (see list below)
* **Value**: `remove`q

**Supported Object Types (Key)**

| Key | Target |
| :--- | :--- |
| `all` | All recognized objects (Objects) and PointClouds |
| `all_objects` | All recognized objects |
| `pointcloud` | PointCloud data |
| `unknown`, `car`, `truck`, `bus`, `trailer`, `motorcycle`, `bicycle`, `pedestrian` | Corresponding recognized objects |

#### 4. Height Configuration (Optional)
You can restrict the removal scope based on height conditions.

This function uses the "height of the centroid (elevation)" of the Lanelet polygon as the reference height. Therefore, please ensure appropriate height (z-coordinate) settings for the polygon's constituent points. Basically, it is recommended to match the height of the road surface (not the sidewalk).
If tags for the height filter are not set, the polygon's height information only affects visualization.

> **Note**: Unlike the horizontal (XY plane) check, the height check considers the "object's occupied area (height range)" rather than just the object's centroid.

| Key | Value Example | Description |
| :--- | :--- | :--- |
| **remove_below_height** | `0.5` | Removes objects located **below** the specified height (m).<br>(e.g., removing weeds or noise near the ground) |
| **remove_above_height** | `2.0` | Removes objects located **above** the specified height (m).<br>(e.g., preventing false detections from signboards or street trees) |

## Lanelet OSM file example

Below is an example of an OSM file description.

```xml
  <way id="89151">
    <nd ref="89141"/>
    <nd ref="89145"/>
    <nd ref="89146"/>
    <nd ref="89150"/>
    <nd ref="89141"/>
    <tag k="type" v="obstacle_removal_area"/>
    <tag k="area" v="yes"/>
    <tag k="unknown" v="remove"/>
    <tag k="pointcloud" v="remove"/>
    <tag k="remove_below_height" v="0.5"/>
    <tag k="remove_above_height" v="2.0"/>
  </way>