#include "morecolordialog.h"
#include "ui_morecolordialog.h"

#include <QGridLayout>

MoreColorDialog::MoreColorDialog(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::MoreColorDialog)
    , m_selectedColor(Qt::red)
{
    ui->setupUi(this);
    setWindowFlags(windowFlags() & ~Qt::WindowContextHelpButtonHint);

    createColorButtons();

    connect(ui->spinR, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &MoreColorDialog::onRGBChanged);
    connect(ui->spinG, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &MoreColorDialog::onRGBChanged);
    connect(ui->spinB, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &MoreColorDialog::onRGBChanged);

    updatePreview();
}

MoreColorDialog::~MoreColorDialog()
{
    delete ui;
}

QColor MoreColorDialog::selectedColor() const
{
    return m_selectedColor;
}

void MoreColorDialog::setCurrentColor(const QColor &color)
{
    m_selectedColor = color;
    ui->spinR->blockSignals(true);
    ui->spinG->blockSignals(true);
    ui->spinB->blockSignals(true);
    ui->spinR->setValue(color.red());
    ui->spinG->setValue(color.green());
    ui->spinB->setValue(color.blue());
    ui->spinR->blockSignals(false);
    ui->spinG->blockSignals(false);
    ui->spinB->blockSignals(false);
    updatePreview();
}

void MoreColorDialog::createColorButtons()
{
    struct ColorEntry {
        QString name;
        QColor color;
    };

    const QList<ColorEntry> palette = {
        // Row 1: Reds & Pinks
        {QStringLiteral("Dark Red"),     QColor(139, 0, 0)},
        {QStringLiteral("Red"),          QColor(255, 0, 0)},
        {QStringLiteral("Orange Red"),   QColor(255, 69, 0)},
        {QStringLiteral("Hot Pink"),     QColor(255, 105, 180)},
        {QStringLiteral("Light Pink"),   QColor(255, 182, 193)},
        {QStringLiteral("Coral"),        QColor(255, 127, 80)},
        {QStringLiteral("Tomato"),       QColor(255, 99, 71)},
        {QStringLiteral("Magenta"),      QColor(255, 0, 255)},

        // Row 2: Oranges & Yellows
        {QStringLiteral("Orange"),       QColor(255, 165, 0)},
        {QStringLiteral("Dark Orange"),  QColor(255, 140, 0)},
        {QStringLiteral("Gold"),         QColor(255, 215, 0)},
        {QStringLiteral("Yellow"),       QColor(255, 255, 0)},
        {QStringLiteral("Light Yellow"), QColor(255, 255, 224)},
        {QStringLiteral("Lemon"),        QColor(255, 250, 205)},
        {QStringLiteral("Amber"),        QColor(255, 191, 0)},
        {QStringLiteral("Apricot"),      QColor(251, 206, 177)},

        // Row 3: Greens
        {QStringLiteral("Dark Green"),   QColor(0, 100, 0)},
        {QStringLiteral("Green"),        QColor(0, 128, 0)},
        {QStringLiteral("Lime Green"),   QColor(50, 205, 50)},
        {QStringLiteral("Lime"),         QColor(0, 255, 0)},
        {QStringLiteral("Spring Green"), QColor(0, 255, 127)},
        {QStringLiteral("Sea Green"),    QColor(46, 139, 87)},
        {QStringLiteral("Olive"),        QColor(128, 128, 0)},
        {QStringLiteral("Mint"),         QColor(144, 238, 144)},

        // Row 4: Blues
        {QStringLiteral("Dark Blue"),    QColor(0, 0, 139)},
        {QStringLiteral("Blue"),         QColor(0, 0, 255)},
        {QStringLiteral("Royal Blue"),   QColor(65, 105, 225)},
        {QStringLiteral("Sky Blue"),     QColor(135, 206, 235)},
        {QStringLiteral("Cyan"),         QColor(0, 255, 255)},
        {QStringLiteral("Teal"),         QColor(0, 206, 209)},
        {QStringLiteral("Navy"),         QColor(0, 39, 102)},
        {QStringLiteral("Steel Blue"),   QColor(70, 130, 180)},

        // Row 5: Purples & Violets
        {QStringLiteral("Indigo"),       QColor(75, 0, 130)},
        {QStringLiteral("Purple"),       QColor(128, 0, 128)},
        {QStringLiteral("Violet"),       QColor(138, 43, 226)},
        {QStringLiteral("Medium Purple"),QColor(147, 112, 219)},
        {QStringLiteral("Orchid"),       QColor(218, 112, 214)},
        {QStringLiteral("Lavender"),     QColor(230, 230, 250)},
        {QStringLiteral("Plum"),         QColor(221, 160, 221)},
        {QStringLiteral("Dark Violet"),  QColor(148, 0, 211)},

        // Row 6: Browns & Earth
        {QStringLiteral("Brown"),        QColor(139, 69, 19)},
        {QStringLiteral("Chocolate"),    QColor(210, 105, 30)},
        {QStringLiteral("Sandy Brown"),  QColor(244, 164, 96)},
        {QStringLiteral("Wheat"),        QColor(245, 222, 179)},
        {QStringLiteral("Beige"),        QColor(245, 245, 220)},
        {QStringLiteral("Sienna"),       QColor(160, 82, 45)},
        {QStringLiteral("Goldenrod"),    QColor(218, 165, 32)},
        {QStringLiteral("Peach"),        QColor(255, 218, 185)},

        // Row 7: Whites, Grays, Blacks
        {QStringLiteral("White"),        QColor(255, 255, 255)},
        {QStringLiteral("Snow"),         QColor(255, 250, 250)},
        {QStringLiteral("Ivory"),        QColor(255, 255, 240)},
        {QStringLiteral("Light Gray"),   QColor(211, 211, 211)},
        {QStringLiteral("Gray"),         QColor(128, 128, 128)},
        {QStringLiteral("Dim Gray"),     QColor(105, 105, 105)},
        {QStringLiteral("Dark Gray"),    QColor(47, 79, 79)},
        {QStringLiteral("Black"),        QColor(0, 0, 0)},
    };

    QGridLayout *grid = ui->gridLayoutBasic;
    const int cols = 8;

    for (int i = 0; i < palette.size(); ++i) {
        QPushButton *btn = new QPushButton(this);
        btn->setMinimumSize(44, 28);
        btn->setMaximumSize(56, 32);
        btn->setToolTip(palette[i].name);
        btn->setStyleSheet(QString(
            "QPushButton {"
            "  background-color: %1;"
            "  border: 2px solid #888;"
            "  border-radius: 3px;"
            "}"
            "QPushButton:hover {"
            "  border: 2px solid #000;"
            "}"
            "QPushButton:checked {"
            "  border: 3px solid #000;"
            "}"
        ).arg(palette[i].color.name()));

        btn->setCheckable(true);
        btn->setProperty("color", palette[i].color);
        connect(btn, &QPushButton::clicked, this, &MoreColorDialog::onColorButtonClicked);

        m_colorButtons.append(btn);
        grid->addWidget(btn, i / cols, i % cols);
    }
}

void MoreColorDialog::onColorButtonClicked()
{
    QPushButton *btn = qobject_cast<QPushButton *>(sender());
    if (!btn) return;

    QColor color = btn->property("color").value<QColor>();
    if (!color.isValid()) return;

    m_selectedColor = color;
    ui->spinR->blockSignals(true);
    ui->spinG->blockSignals(true);
    ui->spinB->blockSignals(true);
    ui->spinR->setValue(color.red());
    ui->spinG->setValue(color.green());
    ui->spinB->setValue(color.blue());
    ui->spinR->blockSignals(false);
    ui->spinG->blockSignals(false);
    ui->spinB->blockSignals(false);

    if (m_lastCheckedButton && m_lastCheckedButton != btn) {
        m_lastCheckedButton->setChecked(false);
    }
    m_lastCheckedButton = btn;
    btn->setChecked(true);

    updatePreview();
}

void MoreColorDialog::onRGBChanged()
{
    m_selectedColor = QColor(ui->spinR->value(), ui->spinG->value(), ui->spinB->value());
    updatePreview();
}

void MoreColorDialog::updatePreview()
{
    QPixmap px(ui->labelPreview->size());
    px.fill(m_selectedColor);
    ui->labelPreview->setPixmap(px);

    QString style = QString("background-color: %1;").arg(m_selectedColor.name());
    ui->labelPreview->setStyleSheet(style);
}
