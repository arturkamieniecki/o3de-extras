/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "AzCore/EBus/Policies.h"
#include "AzCore/std/string/string.h"
#include <AzCore/Component/EntityId.h>
#include <AzCore/EBus/EBus.h>
#include <AzCore/IO/FileIO.h>
#include <AzCore/Interface/Interface.h>
#include <urdf_parser/urdf_parser.h>

namespace ROS2
{
    class RobotImporterRequest : public AZ::EBusTraits
    {
    public:
        AZ_RTTI(RobotImporterRequest, "{36c73a20-ed7a-11ed-a05b-0242ac120003}");
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////

        virtual ~RobotImporterRequest() = default;

        virtual bool generatePrefabFromFile(const AZStd::string filePath, bool importAssetWithUrdf, bool useArticulation) const = 0;
    };

    using RobotImporterRequestBus = AZ::EBus<RobotImporterRequest>;
} // namespace ROS2