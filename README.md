# PCL Transformation

This repo is implementing PCL to PCD, PCL Transformer, PCL to PCD.

## 1. Configure Launch File

```
    <node name="pcd_opener" pkg="pcd_viewer" type="pcd_opener" output="screen">
        <param name="pcd_dir" value="$(find pcd_viewer)/pcd/{pcd_file_name}.pcd" />
    </node>
```

Change {pcd_file_name} into the desired pcd.

```
    <node name="pcd_transformer" pkg="pcd_viewer" type="pcd_transformer" output="screen">
        <param name="pcd_name" value="{saved_pcd_file_name}" />
    </node>
```
change {saved_pcd_file_name} for saved pcd file name, the file will be saved in pcd dir

## 2. Run Launch File

```
roslaunch pcd_viewer transform.launch
```


## 3. Rotate point cloud in y-axis dynamicly

```
rosrun rqt_reconfigure rqt_reconfigure
```
To change degree of rotation, use the slider or box

## 4. Save the transformed PCL to PCD 

```
rosservice call /pcd_transformer/save_pcl
```
