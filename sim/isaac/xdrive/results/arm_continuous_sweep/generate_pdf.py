"""Generate a clean PDF report from the arm torque sweep results."""
import os
from fpdf import FPDF

OUT_DIR = os.path.dirname(os.path.abspath(__file__))
PDF_PATH = os.path.join(OUT_DIR, "sweep_report.pdf")


class Report(FPDF):
    MARGIN = 18
    WHITE = (255, 255, 255)
    BLACK = (30, 30, 30)
    GREY = (110, 110, 110)
    ROW_ALT = (240, 240, 240)
    NOTE_BG = (235, 235, 235)

    def __init__(self):
        super().__init__(orientation="P", unit="mm", format="letter")
        self.set_auto_page_break(auto=False, margin=20)
        self.set_margins(self.MARGIN, self.MARGIN, self.MARGIN)
        self.alias_nb_pages()

    def header(self):
        if self.page_no() == 1:
            return
        self.set_font("Helvetica", "I", 8)
        self.set_text_color(*self.GREY)
        self.cell(0, 6, "FORTIS Arm Torque Sweep Report", align="L")
        self.cell(0, 6, f"Page {self.page_no()}/{{nb}}", align="R", new_x="LMARGIN", new_y="NEXT")
        self.ln(2)

    def footer(self):
        self.set_y(-12)
        self.set_font("Helvetica", "I", 7)
        self.set_text_color(*self.GREY)
        self.cell(0, 5, f"Page {self.page_no()}/{{nb}}", align="C")

    # -- layout helpers --

    def _space_left(self):
        """Mm remaining before bottom margin."""
        return self.h - 20 - self.get_y()

    def _table_height(self, nrows):
        return 6.5 * (nrows + 1) + 2

    def ensure_space(self, needed_mm):
        """Add a new page if needed_mm won't fit."""
        if self._space_left() < needed_mm:
            self.add_page()

    def section(self, title):
        self.ln(3)
        self.set_font("Helvetica", "B", 13)
        self.set_text_color(*self.BLACK)
        self.cell(0, 8, title, new_x="LMARGIN", new_y="NEXT")
        y = self.get_y()
        self.set_draw_color(0, 0, 0)
        self.set_line_width(0.5)
        self.line(self.MARGIN, y, self.w - self.MARGIN, y)
        self.ln(3)

    def subsection(self, title):
        self.ln(2)
        self.set_font("Helvetica", "B", 10.5)
        self.set_text_color(*self.BLACK)
        self.cell(0, 6, title, new_x="LMARGIN", new_y="NEXT")
        self.ln(1)

    def body(self, text):
        self.set_font("Helvetica", "", 9.5)
        self.set_text_color(*self.BLACK)
        self.multi_cell(0, 4.8, text)
        self.ln(1)

    def note(self, text):
        x0, y0 = self.get_x(), self.get_y()
        w = self.w - 2 * self.MARGIN
        self.set_font("Helvetica", "I", 9)
        self.set_text_color(60, 60, 60)
        # estimate height
        h = max(self.get_string_width(text) / (w - 8) * 5 + 10, 12)
        self.set_fill_color(*self.NOTE_BG)
        self.set_draw_color(160, 160, 160)
        self.rect(x0, y0, w, h, style="FD")
        self.set_xy(x0 + 4, y0 + 3)
        self.multi_cell(w - 8, 4.5, text)
        self.set_y(y0 + h + 2)

    def table(self, headers, rows, col_widths=None, bold_col0=True):
        usable = self.w - 2 * self.MARGIN
        n = len(headers)
        if col_widths is None:
            col_widths = [usable / n] * n
        else:
            s = sum(col_widths)
            col_widths = [w / s * usable for w in col_widths]

        row_h = 6.5
        self.ensure_space(self._table_height(len(rows)))

        # header
        self.set_font("Helvetica", "B", 8.5)
        self.set_fill_color(0, 0, 0)
        self.set_text_color(255, 255, 255)
        self.set_draw_color(0, 0, 0)
        for i, h in enumerate(headers):
            align = "L" if i == 0 else "C"
            self.cell(col_widths[i], row_h, f"  {h}" if i == 0 else h,
                      border=1, fill=True, align=align)
        self.ln()

        # rows
        self.set_draw_color(180, 180, 180)
        for ri, row in enumerate(rows):
            self.set_fill_color(*(self.ROW_ALT if ri % 2 == 1 else self.WHITE))
            for ci, val in enumerate(row):
                if ci == 0 and bold_col0:
                    self.set_font("Helvetica", "B", 8.5)
                else:
                    self.set_font("Helvetica", "", 8.5)
                self.set_text_color(*self.BLACK)
                align = "L" if ci == 0 else "C"
                self.cell(col_widths[ci], row_h,
                          f"  {val}" if ci == 0 else str(val),
                          border="LRB", fill=True, align=align)
            self.ln()
        self.ln(2)


def build():
    pdf = Report()

    # ===================== TITLE PAGE =====================
    pdf.add_page()
    pdf.ln(50)
    pdf.set_font("Helvetica", "B", 28)
    pdf.set_text_color(*Report.BLACK)
    pdf.cell(0, 14, "Arm Torque Sweep Report", align="C", new_x="LMARGIN", new_y="NEXT")
    pdf.ln(3)
    pdf.set_font("Helvetica", "", 12)
    pdf.set_text_color(*Report.GREY)
    pdf.cell(0, 7, "FORTIS Tokamak Inspection Robot", align="C", new_x="LMARGIN", new_y="NEXT")
    pdf.ln(14)
    pdf.set_draw_color(0, 0, 0)
    pdf.set_line_width(0.5)
    cx = pdf.w / 2
    pdf.line(cx - 35, pdf.get_y(), cx + 35, pdf.get_y())
    pdf.ln(14)
    pdf.set_font("Helvetica", "", 10)
    pdf.set_text_color(*Report.BLACK)
    for l in [
        "3 arm lengths (36\", 30\", 24\")  x  2 loading (bare / 3 lb payload)",
        "2574 poses per config  |  Step environment (deployment scenario)",
        "Physics torques validated against analytical gravity to < 0.5%",
    ]:
        pdf.cell(0, 6, l, align="C", new_x="LMARGIN", new_y="NEXT")

    # ===================== RESULTS =====================
    pdf.add_page()
    pdf.section("Filtered Results")
    pdf.body(
        "Torques for physically realizable poses on the step environment. "
        "All values in Nm. Peak = worst case for motor sizing. Avg = relevant for power/thermal."
    )
    pdf.table(
        ["Config", "Valid", "J2 Peak", "J3 Peak", "J4 Peak", "J2 Avg", "J3 Avg", "J4 Avg"],
        [
            ["24\" bare",   "1845 / 2574",  "8.50",  "3.08", "0.25", "3.54", "1.88", "0.16"],
            ["24\" loaded", "1845 / 2574", "16.56",  "7.09", "0.93", "6.59", "4.11", "0.59"],
            ["30\" bare",   "1890 / 2574", "10.58",  "3.78", "0.38", "4.50", "2.26", "0.24"],
            ["30\" loaded", "1867 / 2574", "19.30",  "8.80", "1.40", "8.32", "4.92", "0.89"],
            ["36\" bare",   "1880 / 2574", "12.53",  "4.77", "0.50", "5.22", "2.84", "0.32"],
            ["36\" loaded", "1694 / 2574", "19.42", "11.09", "1.87", "8.47", "5.89", "1.19"],
        ],
        col_widths=[18, 22, 14, 14, 14, 14, 14, 14],
    )

    pdf.section("Motor Sizing")
    pdf.body("Design-against peak torques. 36\" loaded is the hardest case.")
    pdf.table(
        ["Config", "J2 Peak", "J3 Peak", "J4 Peak"],
        [
            ["24\" bare",    "8.5 Nm",  "3.1 Nm", "0.3 Nm"],
            ["24\" loaded", "16.6 Nm",  "7.1 Nm", "0.9 Nm"],
            ["30\" bare",   "10.6 Nm",  "3.8 Nm", "0.4 Nm"],
            ["30\" loaded", "19.3 Nm",  "8.8 Nm", "1.4 Nm"],
            ["36\" bare",   "12.5 Nm",  "4.8 Nm", "0.5 Nm"],
            ["36\" loaded", "19.4 Nm", "11.1 Nm", "1.9 Nm"],
        ],
        col_widths=[25, 25, 25, 25],
    )

    pdf.section("Arm Configuration")
    pdf.table(
        ["Parameter", "Value"],
        [
            ["Joint mass (J3, J4)", "629 g each"],
            ["Gripper", "500 g"],
            ["Camera", "98 g (Orbbec Gemini 2, on L1)"],
            ["Link tube", "0.79\" x 0.79\" pultruded CF square"],
            ["Loaded payload", "3 lb (1360 g) at end effector"],
        ],
        col_widths=[40, 60],
        bold_col0=False,
    )
    pdf.body(
        "Camera on L1 affects J2. Gripper at the tip dominates J3/J4. "
        "The 3 lb payload roughly doubles peak torques."
    )

    # ===================== DEFLECTION =====================
    pdf.add_page()
    pdf.section("Link Deflection Under Load")
    pdf.body(
        "CF tube: 25492 square (1.250\" ID, 1.380\" OD, 0.065\" wall, "
        "E = 80.7 GPa, I = 2.42e-8 m4). Linear density 0.226 lb/ft."
    )

    pdf.subsection("Tip Load on L1 (Loaded Configs)")
    pdf.body(
        "Outboard of J2: J3 (629 g) + L2 + J4 (629 g) + L3 + EE (1860 g)."
    )
    pdf.table(
        ["Config", "L2 Mass", "L3 Mass", "Total Load"],
        [
            ["36\" loaded", "~110 g", "~30 g", "3.26 kg (32.0 N)"],
            ["30\" loaded",  "~88 g", "~22 g", "3.23 kg (31.7 N)"],
            ["24\" loaded",  "~73 g", "~15 g", "3.21 kg (31.5 N)"],
        ],
        col_widths=[22, 22, 22, 34],
    )

    pdf.subsection("L1 Deflection")
    pdf.table(
        ["Config", "Force (N)", "Length (m)", "Deflection (mm)"],
        [
            ["36\" loaded", "32.0", "0.432", "0.44"],
            ["30\" loaded", "31.7", "0.381", "0.30"],
            ["24\" loaded", "31.5", "0.254", "0.09"],
        ],
        col_widths=[22, 26, 26, 26],
    )

    pdf.subsection("L2 Deflection")
    pdf.body("Outboard of J3: J4 (629 g) + L3 + EE (1860 g).")
    pdf.table(
        ["Config", "Force (N)", "Length (m)", "Deflection (mm)"],
        [
            ["36\" loaded", "24.5", "0.381", "0.23"],
            ["30\" loaded", "24.4", "0.305", "0.12"],
            ["24\" loaded", "24.3", "0.305", "0.12"],
        ],
        col_widths=[22, 26, 26, 26],
    )

    pdf.subsection("Total EE Sag (Loaded, Stacked)")
    pdf.table(
        ["Config", "L1", "L2", "L3", "Total"],
        [
            ["36\" loaded", "0.44 mm", "0.23 mm", "~0.02 mm", "~0.7 mm"],
            ["30\" loaded", "0.30 mm", "0.12 mm", "~0.01 mm", "~0.4 mm"],
            ["24\" loaded", "0.09 mm", "0.12 mm", "~0.01 mm", "~0.2 mm"],
        ],
        col_widths=[20, 20, 20, 20, 20],
    )
    pdf.body("Under 1 mm worst case. CF tube stiffness is not a concern.")

    # ===================== STABILITY =====================
    pdf.add_page()
    pdf.section("Chassis Stability")
    pdf.body(
        "Static CG projection of chassis (20.4 kg) + arm onto the ground plane, "
        "checked against the wheel support polygon. J1 swept 0-345 deg in 15 deg steps."
    )
    pdf.note(
        "Assumes level chassis. The 4.5\" step tilt will reduce margins on the downhill side. "
        "Step tilt correction is a follow-up."
    )
    pdf.ln(2)
    pdf.table(
        ["Config", "Rejected", "Worst Margin", "Avg Margin", "Tight (< 10 mm)"],
        [
            ["24\" bare",     "0 (0.0%)", "43.0 mm / 1.69\"", "74.2 mm", "0"],
            ["24\" loaded",   "0 (0.0%)", "11.4 mm / 0.45\"", "45.7 mm", "0"],
            ["30\" bare",     "0 (0.0%)", "33.8 mm / 1.33\"", "63.0 mm", "0"],
            ["30\" loaded",  "23 (1.2%)",  "0.5 mm / 0.02\"", "35.5 mm", "139 (7.4%)"],
            ["36\" bare",     "0 (0.0%)", "25.2 mm / 0.99\"", "54.7 mm", "0"],
            ["36\" loaded", "186 (9.9%)",  "0.0 mm / 0.00\"", "28.4 mm", "151 (8.9%)"],
        ],
        col_widths=[20, 18, 25, 20, 22],
    )
    pdf.body(
        "Rejected poses are fully-extended with J1 ~ 90 deg (arm sideways). "
        "24\" is unconditionally stable. 36\" loaded rejects ~10% at full extension."
    )

    # ===================== FILTERING =====================
    pdf.section("Filtering Details")
    pdf.body("A pose is rejected if any check fails:")
    for name, desc in [
        ("Chassis collision", "Link sample points inside chassis bounding box (10 mm margin)."),
        ("Self-collision (L3-L1)", "L3 and L1 segments closer than 15 mm."),
        ("Self-collision (L3-J1)", "L3 enters J1 base volume."),
        ("Floor collision", "Any joint or link midpoint below floor (10 mm margin)."),
        ("Tipping", "CG projection outside wheel polygon at any J1 angle."),
    ]:
        pdf.set_font("Helvetica", "B", 9.5)
        pdf.set_text_color(*Report.BLACK)
        pdf.cell(pdf.get_string_width(name + "  "), 5, name + "  ")
        pdf.set_font("Helvetica", "", 9.5)
        pdf.multi_cell(0, 5, desc)
        pdf.ln(0.5)

    pdf.ln(1)
    pdf.subsection("Rejection Breakdown")
    pdf.table(
        ["Arm", "Loading", "Valid", "Chassis", "Self L3-L1", "Self L3-J1", "Floor", "Tip"],
        [
            ["24\"", "bare",   "1845 (71.7%)", "487", "100", "0",   "142", "0"],
            ["24\"", "loaded", "1845 (71.7%)", "487", "100", "0",   "142", "0"],
            ["30\"", "bare",   "1890 (73.4%)", "380", "100", "0",   "204", "0"],
            ["30\"", "loaded", "1867 (72.5%)", "380", "100", "0",   "204", "23"],
            ["36\"", "bare",   "1880 (73.0%)", "353",  "74", "4",   "263", "0"],
            ["36\"", "loaded", "1694 (65.8%)", "353",  "74", "4",   "263", "186"],
        ],
        col_widths=[10, 14, 22, 14, 16, 16, 12, 10],
        bold_col0=False,
    )

    # ===================== RAW + FLAT (APPENDIX) =====================
    pdf.add_page()
    pdf.section("Appendix: Raw and Flat Data")

    pdf.subsection("Raw Torques (Unfiltered, Step)")
    pdf.body("All 2574 poses including impossible configurations.")
    pdf.table(
        ["Config", "J2 Peak", "J3 Peak", "J4 Peak", "J2 Avg", "J3 Avg", "J4 Avg"],
        [
            ["24\" bare",    "8.50",  "3.08", "0.25", "3.78", "1.81", "0.16"],
            ["24\" loaded", "16.56",  "7.09", "0.93", "6.86", "3.95", "0.59"],
            ["30\" bare",   "10.58",  "3.78", "0.38", "4.73", "2.18", "0.24"],
            ["30\" loaded", "20.65",  "8.80", "1.40", "8.53", "4.75", "0.89"],
            ["36\" bare",   "12.53",  "4.77", "0.50", "5.45", "2.74", "0.32"],
            ["36\" loaded", "24.60", "11.09", "1.88", "9.89", "5.96", "1.18"],
        ],
        col_widths=[20, 16, 16, 16, 16, 16, 16],
    )

    pdf.subsection("Flat Environment (Filtered)")
    pdf.note(
        "NOT reliable for motor sizing. Chassis slides on flat ground during arm teleport, "
        "inflating measured torques by 100-200%. Use step data for all design decisions."
    )
    pdf.ln(2)
    pdf.table(
        ["Config", "J2 Peak", "J3 Peak", "J4 Peak", "J2 Avg", "J3 Avg", "J4 Avg"],
        [
            ["24\" bare",   "17.37", "10.55", "0.75", "3.67", "1.93", "0.16"],
            ["24\" loaded", "22.80", "11.63", "1.67", "6.71", "4.15", "0.59"],
            ["30\" bare",   "13.17",  "5.86", "0.71", "4.52", "2.27", "0.24"],
            ["30\" loaded", "33.73", "16.95", "2.74", "8.53", "5.00", "0.89"],
            ["36\" bare",   "25.39", "11.74", "1.19", "5.74", "3.05", "0.34"],
            ["36\" loaded", "37.08", "24.25", "4.21", "10.01", "6.32", "1.20"],
        ],
        col_widths=[20, 16, 16, 16, 16, 16, 16],
    )

    # ===================== METHOD (last) =====================
    pdf.add_page()
    pdf.section("Method")
    pdf.body(
        "Isaac Sim headless, 360 Hz. Collision disabled on arm links so every joint combo is reachable. "
        "Each pose: teleport to target angles, PD holds ~0.4 s, measure torques ~0.1 s. "
        "Post-filter removes impossible poses (chassis/self/floor collision, tipping)."
    )
    pdf.body(
        "All 6 step configs: 2574/2574 converged, 0 NaN. "
        "Physics torques match analytical gravity to < 0.5%."
    )

    pdf.output(PDF_PATH)
    print(f"PDF written to {PDF_PATH}")


if __name__ == "__main__":
    build()
