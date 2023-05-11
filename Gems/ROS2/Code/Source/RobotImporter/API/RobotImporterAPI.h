/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "AzCore/IO/Path/Path_fwd.h"
#include <AzToolsFramework/API/ToolsApplicationAPI.h>

namespace ROS2
{
    class RobotImporterAPI
    {
    public:
        // This function returns true on success, and false on error.
        static bool generatePrefabFromFile(const AZ::IO::Path filePath, bool importAssetWithUrdf, bool useArticulation);

    private:
        static bool CheckCyclicalDependency(AZ::IO::Path importedPrefabPath);
    };
} // namespace ROS2