#include "housing_selector_widget.h"
#include "calibmar/core/housing_import.h"

namespace calibmar {

  HousingSelectorWidget::HousingSelectorWidget(QWidget* parent) : QGroupBox(parent) {
    setTitle("Housing Interface");
    housing_model_ = new HousingWidget(this);
    initial_parameters_edit_ = new QLineEdit(this);
    initial_parameters_edit_->setPlaceholderText("Initial Parameters");

    QPushButton* load_button = new QPushButton("Load...", this);
    load_button->setToolTip("Load a housing yaml (housing, thickness, glass, water, ...) or a previous housing calibration");
    connect(load_button, &QPushButton::released, this, &HousingSelectorWidget::LoadHousingYaml);
    QHBoxLayout* parameters_layout = new QHBoxLayout();
    parameters_layout->addWidget(initial_parameters_edit_);
    parameters_layout->addWidget(load_button);

    QVBoxLayout* layout = new QVBoxLayout(this);

    layout->addWidget(housing_model_);
    layout->addLayout(parameters_layout);
  }

  void HousingSelectorWidget::LoadHousingYaml() {
    QString path = QFileDialog::getOpenFileName(this, "Housing", QString(), "YAML (*.yaml *.yml);;All files (*)");
    if (path.isEmpty()) {
      return;
    }
    try {
      housing_import::ImportedHousing housing = housing_import::ReadHousing(path.toStdString());
      QStringList params;
      for (double p : housing.params) {
        params << QString::number(p, 'g', 10);
      }
      SetHousingOptions(std::make_pair(housing.housing_interface, params.join(", ").toStdString()));
    }
    catch (const std::exception& ex) {
      QMessageBox::warning(this, "Housing", QString::fromStdString(ex.what()));
    }
  }

  std::optional<std::pair<HousingInterfaceType, std::string>> HousingSelectorWidget::HousingOptions() {
    auto housing_model = housing_model_->HousingType();
    if (!housing_model.has_value()) {
      return {};
    }
    else {
      return std::make_pair(*housing_model, initial_parameters_edit_->text().toStdString());
    }
  }

  void HousingSelectorWidget::SetHousingOptions(const std::optional<std::pair<HousingInterfaceType, std::string>>& options) {
    if (options.has_value()) {
      housing_model_->SetHousingType(options->first);
      initial_parameters_edit_->setText(QString::fromStdString(options->second));
    }
    else {
      housing_model_->SetHousingType({});
      initial_parameters_edit_->clear();
    }
  }
}