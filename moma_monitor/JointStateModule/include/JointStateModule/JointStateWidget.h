//
// Created by acey on 24.10.23.
//

#ifndef MOMA_MONITOR_JOINTSTATEWIDGET_H
#define MOMA_MONITOR_JOINTSTATEWIDGET_H

#include "state_monitor_sdk/ui/widgets/Dial.h"
#include <QFrame>
#include <QLabel>

class JointStateWidget : public QFrame {
public:
  explicit JointStateWidget(QWidget* parent = nullptr);
  void setRange(double min, double max);
  void setCurrent(double val);
  ~JointStateWidget() override;
private:
  void buildLayout();
  QLabel* label_;
  Dial* dial_;
  DialHand* pos_hand_;
};

#endif // MOMA_MONITOR_JOINTSTATEWIDGET_H
