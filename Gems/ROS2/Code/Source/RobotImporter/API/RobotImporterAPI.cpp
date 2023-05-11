/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RobotImporterAPI.h"
#include "../URDF/URDFPrefabMaker.h"
#include "../URDF/UrdfParser.h"
#include "../Utils/FilePath.h"
#include "../Utils/RobotImporterUtils.h"
#include "../xacro/XacroUtils.h"
#include "Prefab/PrefabFocusInterface.h"
#include <AzCore/IO/FileIO.h>
#include <AzCore/Utils/Utils.h>
#include <stdlib.h>

namespace ROS2
{
    bool RobotImporterAPI::generatePrefabFromFile(const AZ::IO::Path filePath, bool importAssetWithUrdf, bool useArticulation)
    {
        Utils::xacro::Params params;
        urdf::ModelInterfaceSharedPtr parsedUrdf;

        AZStd::unique_ptr<URDFPrefabMaker> prefabMaker;
        AZStd::shared_ptr<Utils::UrdfAssetMap> urdfAssetsMapping;
        AZStd::unordered_set<AZStd::string> meshNames;

        if (filePath.empty())
        {
            return false;
        }

        if (Utils::IsFileXacro(filePath))
        {
            Utils::xacro::ExecutionOutcome outcome = Utils::xacro::ParseXacro(filePath.String(), params);
            if (outcome)
            {
                parsedUrdf = outcome.m_urdfHandle;
            }
            else
            {
                return false;
            }
        }
        else if (Utils::IsFileUrdf(filePath))
        {
            parsedUrdf = UrdfParser::ParseFromFile(filePath.Native());
        }
        else
        {
            return false;
        }

        const auto log = UrdfParser::GetUrdfParsingLog();

        if (!parsedUrdf)
        {
            return false;
        }
        meshNames = Utils::GetMeshesFilenames(parsedUrdf->getRoot(), true, true);

        auto collidersNames = Utils::GetMeshesFilenames(parsedUrdf->getRoot(), false, true);
        auto visualNames = Utils::GetMeshesFilenames(parsedUrdf->getRoot(), true, false);

        std::cout << importAssetWithUrdf << std::endl;
        if (importAssetWithUrdf)
        {
            std::cout << "part 1" << std::endl;
            urdfAssetsMapping = AZStd::make_shared<Utils::UrdfAssetMap>(
                Utils::CopyAssetForURDFAndCreateAssetMap(meshNames, filePath.String(), collidersNames, visualNames));
        }
        else
        {
            std::cout << "part 2" << std::endl;
            urdfAssetsMapping = AZStd::make_shared<Utils::UrdfAssetMap>(Utils::FindAssetsForUrdf(meshNames, filePath.String()));
            for (const AZStd::string& meshPath : meshNames)
            {
                if (urdfAssetsMapping->contains(meshPath))
                {
                    const auto& asset = urdfAssetsMapping->at(meshPath);
                    bool visual = visualNames.contains(meshPath);
                    bool collider = collidersNames.contains(meshPath);
                    Utils::createSceneManifest(asset.m_availableAssetInfo.m_sourceAssetGlobalPath, collider, visual);
                }
            }
        }


        bool allAssetProcessed = false;
        do
        {
            AZStd::this_thread::sleep_for(AZStd::chrono::milliseconds(300));
            allAssetProcessed = false;
            for (const auto& [name, asset] : *urdfAssetsMapping)
            {
                auto sourceAssetFullPath = asset.m_availableAssetInfo.m_sourceAssetGlobalPath;
                if (sourceAssetFullPath.empty()){
                    AZ_Printf("FooTest", "asset %s is has no sourceAssetFullPath", name.c_str());
                    continue;
                }
                AZ_Printf("FooTest", "checking asset %s", sourceAssetFullPath.c_str());
                using namespace AzToolsFramework;
                using namespace AzToolsFramework::AssetSystem;
                AZ::Outcome<AssetSystem::JobInfoContainer> result = AZ::Failure(); 
                AssetSystemJobRequestBus::BroadcastResult(
                    result, &AssetSystemJobRequestBus::Events::GetAssetJobsInfo, sourceAssetFullPath, true);
                JobInfoContainer& allJobs = result.GetValue();
                for (const JobInfo& job : allJobs)
                {
                    if (job.m_status == JobStatus::Queued || job.m_status == JobStatus::InProgress)
                    {
                        AZ_Printf("FooTest", "asset %s is being processed", sourceAssetFullPath.c_str());
                        allAssetProcessed = true;
                    }
                    else{
                        AZ_Printf("FooTest", "asset %s is done", sourceAssetFullPath.c_str());
                    }
                }
            }
        }while(allAssetProcessed);

//        for (int i = 0; i < m_assetsUuids.size(); i++)
//        {
//            const AZ::Uuid& assetUuid = m_assetsUuids[i];
//            const AZStd::string& sourceAssetFullPath = m_assetsPaths[i];
//            if (!m_assetsUuidsFinished.contains(assetUuid))
//            {
//                using namespace AzToolsFramework;
//                using namespace AzToolsFramework::AssetSystem;
//
//                AZ::Outcome<AssetSystem::JobInfoContainer> result = AZ::Failure();
//                AssetSystemJobRequestBus::BroadcastResult(
//                    result, &AssetSystemJobRequestBus::Events::GetAssetJobsInfo, sourceAssetFullPath, true);
//                bool allFinished = true;
//                bool failed = false;
//                JobInfoContainer& allJobs = result.GetValue();
//                for (const JobInfo& job : allJobs)
//                {
//                    if (job.m_status == JobStatus::Queued || job.m_status == JobStatus::InProgress)
//                    {
//                        allFinished = false;
//                    }
//                    if (job.m_status == JobStatus::Failed)
//                    {
//                        failed = true;
//                        m_failedCount++;
//                    }
//                }
//                if (allFinished)
//                {
//                    if (!failed)
//                    {
//                        const AZStd::string productRelPathVisual = Utils::GetModelProductAsset(assetUuid);
//                        const AZStd::string productRelPathCollider = Utils::GetPhysXMeshProductAsset(assetUuid);
//                        QString text = QString::fromUtf8(productRelPathVisual.data(), productRelPathVisual.size()) + " " +
//                            QString::fromUtf8(productRelPathCollider.data(), productRelPathCollider.size());
//                        m_table->setItem(i, 4, createCell(true, text));
//                        m_table->item(i, 4)->setIcon(m_okIcon);
//                    }
//                    else
//                    {
//                        m_table->setItem(i, 4, createCell(false, tr("Failed")));
//                        m_table->item(i, 4)->setIcon(m_failureIcon);
//                    }
//                    m_assetsUuidsFinished.insert(assetUuid);
//                }
//            }
//        }
        // sleep(5);

        AZStd::string prefabName = AZStd::string(parsedUrdf->getName().c_str(), parsedUrdf->getName().size()) + ".prefab";

        const AZ::IO::Path prefabPathRelative(AZ::IO::Path("Assets") / "Importer" / prefabName);
        const AZ::IO::Path prefabPath(AZ::IO::Path(AZ::Utils::GetProjectPath()) / prefabPathRelative);
        // bool fileExists = AZ::IO::FileIOBase::GetInstance()->Exists(prefabPath.c_str());

        if (CheckCyclicalDependency(prefabPathRelative))
        {
            return false;
        }

        prefabMaker =
            AZStd::make_unique<URDFPrefabMaker>(filePath.String(), parsedUrdf, prefabPath.String(), urdfAssetsMapping, useArticulation);

        auto prefabOutcome = prefabMaker->CreatePrefabFromURDF();

        if (!prefabOutcome.IsSuccess())
        {
            return false;
        }

        prefabMaker.reset();

        return true;
    }

    bool RobotImporterAPI::CheckCyclicalDependency(AZ::IO::Path importedPrefabPath)
    {
        AzFramework::EntityContextId contextId;
        AzFramework::EntityIdContextQueryBus::BroadcastResult(contextId, &AzFramework::EntityIdContextQueryBus::Events::GetOwningContextId);

        auto focusInterface = AZ::Interface<AzToolsFramework::Prefab::PrefabFocusInterface>::Get();

        if (!focusInterface)
        {
            return true;
        }

        auto focusedPrefabInstance = focusInterface->GetFocusedPrefabInstance(contextId);

        if (!focusedPrefabInstance)
        {
            return true;
        }

        auto focusPrefabFilename = focusedPrefabInstance.value().get().GetTemplateSourcePath();

        if (focusPrefabFilename == importedPrefabPath)
        {
            return true;
        }

        return false;
    }
} // namespace ROS2