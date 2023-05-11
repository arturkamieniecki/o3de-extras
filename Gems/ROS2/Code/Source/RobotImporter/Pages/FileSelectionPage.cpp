/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "FileSelectionPage.h"
#include <AzCore/Utils/Utils.h>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <API/RobotImporterAPI.h>

namespace ROS2
{
    FileSelectionPage::FileSelectionPage(QWizard* parent)
        : QWizardPage(parent)
    {
        m_fileDialog = new QFileDialog(this);
        m_fileDialog->setDirectory(QString::fromUtf8(AZ::Utils::GetProjectPath().data()));
        m_fileDialog->setNameFilter("URDF, XACRO (*.urdf *.xacro)");
        m_button = new QPushButton("...", this);
        m_textEdit = new QLineEdit("", this);
        m_copyFiles = new QCheckBox(tr("Import meshes during URDF load"), this);
        m_testButton = new QPushButton(tr("Test"), this);
        m_copyFiles->setCheckState(Qt::CheckState::Checked);
        setTitle(tr("Load URDF file"));
        QVBoxLayout* layout = new QVBoxLayout;
        layout->addStretch();
        layout->addWidget(new QLabel(tr("URDF file path to load : "), this));
        QHBoxLayout* layout_in = new QHBoxLayout;
        layout_in->addWidget(m_button);
        layout_in->addWidget(m_textEdit);
        layout->addLayout(layout_in);
        layout->addWidget(m_copyFiles);
        layout->addWidget(m_testButton);
        layout->addStretch();
        this->setLayout(layout);
        connect(m_button, &QPushButton::pressed, this, &FileSelectionPage::onLoadButtonPressed);
        connect(m_fileDialog, &QFileDialog::fileSelected, this, &FileSelectionPage::onFileSelected);
        connect(m_textEdit, &QLineEdit::textChanged, this, &FileSelectionPage::onTextChanged);
        connect(m_testButton, &QPushButton::pressed, this, &FileSelectionPage::onTestButtonPressed);
        FileSelectionPage::onTextChanged(m_textEdit->text());
    }

    void FileSelectionPage::onLoadButtonPressed()
    {
        m_fileDialog->show();
    }

    void FileSelectionPage::onFileSelected(const QString& file)
    {
        m_textEdit->setText(file);
    }

    void FileSelectionPage::onTextChanged(const QString& text)
    {
        m_fileExists = QFileInfo::exists(text);
        emit completeChanged();
    }

    bool FileSelectionPage::isComplete() const
    {
        return m_fileExists;
    }

    bool FileSelectionPage::getIfCopyAssetsDuringUrdfImport() const
    {
        return m_copyFiles->isChecked();
    }

    void FileSelectionPage::onTestButtonPressed()
    {
        AZ_Printf("ROS2", "Test loading with API %s \n", m_textEdit->text().toUtf8().data());
        ROS2::RobotImporterAPI::generatePrefabFromFile(m_textEdit->text().toUtf8().data(), m_copyFiles->isChecked(), false);

    }
} // namespace ROS2
