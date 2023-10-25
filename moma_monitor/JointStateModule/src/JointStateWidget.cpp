//
// Created by acey on 24.10.23.
//

#include "JointStateWidget.h"

#include <QVBoxLayout>
#include <iostream>
#include <state_monitor_sdk/ui/defaults.h>

JointStateWidget::JointStateWidget(QWidget* parent) : QFrame(parent) {
  setStyleSheet("border: none;");
  dial_ = new Dial();
  dial_->setDialRange(-1, 1);
  label_ = new QLabel("0 rad");
  label_->setStyleSheet(QUi::LabelDefault);
  label_->setFixedHeight(20);

  pos_hand_ = new DialHand(0, Qt::darkGray);
  dial_->addHand(pos_hand_);
  buildLayout();
}

void JointStateWidget::buildLayout() {
  auto *layout = new QVBoxLayout();
  layout->setSpacing(5);
  layout->setMargin(10);
  layout->addWidget(dial_);
  layout->addWidget(label_);

  dial_->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
  dial_->setMinimumWidth(minimumWidth());
  setLayout(layout);
}


void JointStateWidget::setCurrent(double val) {
  label_->setText(QString::number(val) + " rad");
  pos_hand_->setValue(val);
}

JointStateWidget::~JointStateWidget() {
  delete dial_;
}

void JointStateWidget::setRange(double min, double max) {
  dial_->setDialRange(min, max);
}
