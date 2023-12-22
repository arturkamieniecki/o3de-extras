# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT

set(FILES
    ../Assets/Editor/Images/Icons/ROS2.qrc
    ../Assets/Editor/Images/Icons/ROS_import_icon.svg
    Source/Camera/ROS2CameraSensorEditorComponent.cpp
    Source/Camera/ROS2CameraSensorEditorComponent.h
    Source/Camera/ROS2EditorCameraSystemComponent.cpp
    Source/Camera/ROS2EditorCameraSystemComponent.h
    Source/Georeference/GeoreferenceLevelEditorComponent.cpp
    Source/Georeference/GeoreferenceLevelEditorComponent.h
    Source/Lidar/LidarRegistrarEditorSystemComponent.cpp
    Source/Lidar/LidarRegistrarEditorSystemComponent.h
    Source/Manipulation/JointsManipulationEditorComponent.cpp
    Source/Manipulation/JointsManipulationEditorComponent.h
    Source/RobotImporter/FixURDF/FixURDF.cpp
    Source/RobotImporter/FixURDF/FixURDF.h
    Source/RobotImporter/Pages/CheckAssetPage.cpp
    Source/RobotImporter/Pages/CheckAssetPage.h
    Source/RobotImporter/Pages/CheckUrdfPage.cpp
    Source/RobotImporter/Pages/CheckUrdfPage.h
    Source/RobotImporter/Pages/FileSelectionPage.cpp
    Source/RobotImporter/Pages/FileSelectionPage.h
    Source/RobotImporter/Pages/PrefabMakerPage.cpp
    Source/RobotImporter/Pages/PrefabMakerPage.h
    Source/RobotImporter/Pages/IntroPage.cpp
    Source/RobotImporter/Pages/IntroPage.h
    Source/RobotImporter/Pages/XacroParamsPage.cpp
    Source/RobotImporter/Pages/XacroParamsPage.h
    Source/RobotImporter/RobotImporterWidget.cpp
    Source/RobotImporter/RobotImporterWidget.h
    Source/RobotImporter/ROS2RobotImporterEditorSystemComponent.cpp
    Source/RobotImporter/ROS2RobotImporterEditorSystemComponent.h
    Source/RobotImporter/SDFormat/Hooks/ROS2CameraSensorHook.cpp
    Source/RobotImporter/SDFormat/Hooks/ROS2GNSSSensorHook.cpp
    Source/RobotImporter/SDFormat/Hooks/ROS2ImuSensorHook.cpp
    Source/RobotImporter/SDFormat/Hooks/ROS2LidarSensorHook.cpp
    Source/RobotImporter/SDFormat/ROS2SensorHooksUtils.cpp
    Source/RobotImporter/SDFormat/ROS2SensorHooksUtils.h
    Source/RobotImporter/SDFormat/ROS2SensorHooks.h
    Source/RobotImporter/URDF/ArticulationsMaker.cpp
    Source/RobotImporter/URDF/ArticulationsMaker.h
    Source/RobotImporter/URDF/CollidersMaker.cpp
    Source/RobotImporter/URDF/CollidersMaker.h
    Source/RobotImporter/URDF/InertialsMaker.cpp
    Source/RobotImporter/URDF/InertialsMaker.h
    Source/RobotImporter/URDF/JointsMaker.cpp
    Source/RobotImporter/URDF/JointsMaker.h
    Source/RobotImporter/URDF/PrefabMakerUtils.cpp
    Source/RobotImporter/URDF/PrefabMakerUtils.h
    Source/RobotImporter/URDF/SensorsMaker.cpp
    Source/RobotImporter/URDF/SensorsMaker.h
    Source/RobotImporter/URDF/UrdfParser.cpp
    Source/RobotImporter/URDF/UrdfParser.h
    Source/RobotImporter/URDF/URDFPrefabMaker.cpp
    Source/RobotImporter/URDF/URDFPrefabMaker.h
    Source/RobotImporter/URDF/VisualsMaker.cpp
    Source/RobotImporter/URDF/VisualsMaker.h
    Source/RobotImporter/xacro/XacroUtils.cpp
    Source/RobotImporter/xacro/XacroUtils.h
    Source/RobotImporter/Utils/DefaultSolverConfiguration.h
    Source/RobotImporter/Utils/ErrorUtils.cpp
    Source/RobotImporter/Utils/ErrorUtils.h
    Source/RobotImporter/Utils/FilePath.cpp
    Source/RobotImporter/Utils/FilePath.h
    Source/RobotImporter/Utils/RobotImporterUtils.cpp
    Source/RobotImporter/Utils/RobotImporterUtils.h
    Source/RobotImporter/Utils/SourceAssetsStorage.cpp
    Source/RobotImporter/Utils/SourceAssetsStorage.h
    Source/RobotImporter/Utils/TypeConversions.cpp
    Source/RobotImporter/Utils/TypeConversions.h
    Source/ROS2GemUtilities.cpp
    Source/Spawner/ROS2SpawnerEditorComponent.cpp
    Source/Spawner/ROS2SpawnerEditorComponent.h
    Source/Spawner/ROS2SpawnPointEditorComponent.cpp
    Source/Spawner/ROS2SpawnPointEditorComponent.h
    Source/SdfAssetBuilder/SdfAssetBuilder.cpp
    Source/SdfAssetBuilder/SdfAssetBuilder.h
    Source/SdfAssetBuilder/SdfAssetBuilderSettings.cpp
    Source/SdfAssetBuilder/SdfAssetBuilderSettings.h
    Source/SdfAssetBuilder/SdfAssetBuilderSystemComponent.cpp
    Source/SdfAssetBuilder/SdfAssetBuilderSystemComponent.h
    Source/Frame/ROS2FrameEditorComponent.cpp
    Source/Frame/ROS2FrameSystemComponent.cpp
    Source/Frame/ROS2FrameSystemComponent.h
    Source/Frame/ROS2FrameSystemBus.h
    Source/SystemComponents/ROS2EditorSystemComponent.cpp
    Source/SystemComponents/ROS2EditorSystemComponent.h
)
