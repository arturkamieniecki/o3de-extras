/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Entity.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/smart_ptr/shared_ptr.h>
#include <qcombobox.h>
#if !defined(Q_MOC_RUN)
#include <AzCore/Math/Crc.h>
#include <AzCore/std/string/string.h>
#include <QCheckBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QString>
#include <QTextEdit>
#include <QWizardPage>
#endif

namespace ROS2
{
    class RobotImporterWidget;
    class PrefabMakerPage : public QWizardPage
    {
        Q_OBJECT
    public:
        explicit PrefabMakerPage(RobotImporterWidget* parent);
        void setProposedPrefabName(const AZStd::string prefabName);
        AZStd::string getPrefabName() const;
        void reportProgress(const AZStd::string& progressForUser);
        void setSuccess(bool success);
        bool isComplete() const override;
        bool IsUseArticulations() const;
        AZStd::shared_ptr<AZ::Transform> getSelectedSpawnPoint() const;
    Q_SIGNALS:
        void onCreateButtonPressed();

    private:
        bool m_success;
        QLineEdit* m_prefabName;
        QPushButton* m_createButton;
        QTextEdit* m_log;
        QCheckBox* m_useArticulation;
        QComboBox* m_spawnPointsList;
        AZStd::vector<AZStd::pair<AZStd::string, AZStd::shared_ptr<AZ::Transform>>> m_spawnPointsInfos;
        RobotImporterWidget* m_parentImporterWidget;
    };
} // namespace ROS2
