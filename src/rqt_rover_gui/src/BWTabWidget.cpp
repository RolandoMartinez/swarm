#include "BWTabWidget.h"

#include <QPalette>
#include <QTabBar>

/*!
<<<<<<< HEAD
 * Constructor. This function sets the color of the tabs at the top of the GUI.
 * The stylesheets method does not allow changing the color of tab bars.
 */
BWTabWidget::BWTabWidget(QWidget *p) : QTabWidget(p) {
  QPalette pal = tabBar()->palette();

  pal.setColor(QPalette::WindowText, QColor(255,255,255));
=======
 *  Constructor. This function sets the color of the tabs at the
 *  top of the GUI. The stylesheets method does not allow changing
 *  the color of tab bars.
 */
BWTabWidget::BWTabWidget(QWidget *p) : QTabWidget(p)
{
  QPalette pal = tabBar()->palette();
  pal.setColor(QPalette::WindowText, QColor(255,255,255));


>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
  pal.setColor(QPalette::Window, QColor(0,0,0));
  pal.setColor(QPalette::Button, QColor(0,0,0));
  pal.setColor(QPalette::ButtonText, QColor(0,0,0));
  pal.setColor(QPalette::Base, QColor(0, 0, 0));
  pal.setColor(QPalette::AlternateBase, QColor(0, 0, 0));
  pal.setColor(QPalette::Text, Qt::black);
<<<<<<< HEAD

=======
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
  tabBar()->setPalette(pal);
}
