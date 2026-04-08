"""Generate a clean PDF report from the arm torque sweep results."""
import os
from fpdf import FPDF

OUT_DIR = os.path.dirname(os.path.abspath(__file__))
PDF_PATH = os.path.join(OUT_DIR, "sweep_report.pdf")


class Report(FPDF):
    MARGIN = 18
    ACCENT = (33, 76, 134)       # dark blue
    ACCENT_LIGHT = (220, 232, 245)  # light blue fill
    ROW_ALT = (245, 247, 250)    # very light grey
    WHITE = (255, 255, 255)
    BLACK = (30, 30, 30)
    GREY = (110, 110, 110)
    WARN_BG = (255, 243, 224)    # light orange
    WARN_BORDER = (230, 160, 60)

    def __init__(self):
        super().__init__(orientation="P", unit="mm", format="letter")
        self.set_auto_page_break(auto=True, margin=20)
        self.set_margins(self.MARGIN, self.MARGIN, self.MARGIN)
        self.alias_nb_pages()

    # -- chrome --
    def header(self):
        if self.page_no() == 1:
            return
        self.set_font("Helvetica", "I", 8)
        self.set_text_color(*self.GREY)
        self.cell(0, 6, "FORTIS Arm Torque Sweep Report", align="L")
        self.cell(0, 6, f"Page {self.page_no()}/{{nb}}", align="R", new_x="LMARGIN", new_y="NEXT")
        self.ln(2)

    def footer(self):
        self.set_y(-14)
        self.set_font("Helvetica", "I", 7)
        self.set_text_color(*self.GREY)
        self.cell(0, 5, "Generated from Isaac Sim collision-free sweep data", align="C")

    # -- helpers --
    def section(self, title):
        self.ln(4)
        self.set_font("Helvetica", "B", 14)
        self.set_text_color(*self.ACCENT)
        self.cell(0, 8, title, new_x="LMARGIN", new_y="NEXT")
        # underline
        y = self.get_y()
        self.set_draw_color(*self.ACCENT)
        self.set_line_width(0.6)
        self.line(self.MARGIN, y, self.w - self.MARGIN, y)
        self.ln(4)

    def subsection(self, title):
        self.ln(2)
        self.set_font("Helvetica", "B", 11)
        self.set_text_color(*self.BLACK)
        self.cell(0, 7, title, new_x="LMARGIN", new_y="NEXT")
        self.ln(1)

    def body(self, text):
        self.set_font("Helvetica", "", 9.5)
        self.set_text_color(*self.BLACK)
        self.multi_cell(0, 4.8, text)
        self.ln(1)

    def note(self, text):
        """Yellow/orange callout box."""
        self.set_fill_color(*self.WARN_BG)
        self.set_draw_color(*self.WARN_BORDER)
        x0 = self.get_x()
        y0 = self.get_y()
        w = self.w - 2 * self.MARGIN
        self.set_font("Helvetica", "I", 9)
        self.set_text_color(100, 70, 10)
        # measure height
        h = self.get_string_width(text) / (w - 8) * 5 + 10
        h = max(h, 12)
        self.rect(x0, y0, w, h, style="FD")
        self.set_xy(x0 + 4, y0 + 3)
        self.multi_cell(w - 8, 4.5, text)
        self.set_y(y0 + h + 2)

    def table(self, headers, rows, col_widths=None, bold_col0=True, highlight_max_row=False):
        usable = self.w - 2 * self.MARGIN
        n = len(headers)
        if col_widths is None:
            col_widths = [usable / n] * n
        else:
            # scale to fit
            s = sum(col_widths)
            col_widths = [w / s * usable for w in col_widths]

        row_h = 6.5

        # check if table fits on current page, if not add page
        needed = row_h * (len(rows) + 1) + 4
        if self.get_y() + needed > self.h - 25:
            self.add_page()

        # header row
        self.set_font("Helvetica", "B", 8.5)
        self.set_fill_color(*self.ACCENT)
        self.set_text_color(*self.WHITE)
        self.set_draw_color(*self.ACCENT)
        for i, h in enumerate(headers):
            align = "L" if i == 0 else "C"
            self.cell(col_widths[i], row_h, f"  {h}" if i == 0 else h,
                      border=1, fill=True, align=align)
        self.ln()

        # data rows
        self.set_draw_color(200, 200, 200)
        for ri, row in enumerate(rows):
            if ri % 2 == 1:
                self.set_fill_color(*self.ROW_ALT)
            else:
                self.set_fill_color(*self.WHITE)
            fill = True
            for ci, val in enumerate(row):
                if ci == 0 and bold_col0:
                    self.set_font("Helvetica", "B", 8.5)
                    self.set_text_color(*self.BLACK)
                else:
                    self.set_font("Helvetica", "", 8.5)
                    self.set_text_color(*self.BLACK)
                align = "L" if ci == 0 else "C"
                self.cell(col_widths[ci], row_h,
                          f"  {val}" if ci == 0 else str(val),
                          border="LR", fill=fill, align=align)
            self.ln()

        # bottom border
        self.set_draw_color(*self.ACCENT)
        self.set_line_width(0.4)
        y = self.get_y()
        self.line(self.MARGIN, y, self.MARGIN + sum(col_widths), y)
        self.ln(2)


def build():
    pdf = Report()

    # ===================== TITLE PAGE =====================
    pdf.add_page()
    pdf.ln(45)
    pdf.set_font("Helvetica", "B", 28)
    pdf.set_text_color(*Report.ACCENT)
    pdf.cell(0, 14, "Arm Torque Sweep Report", align="C", new_x="LMARGIN", new_y="NEXT")
    pdf.ln(4)
    pdf.set_font("Helvetica", "", 13)
    pdf.set_text_color(*Report.GREY)
    pdf.cell(0, 7, "FORTIS Tokamak Inspection Robot", align="C", new_x="LMARGIN", new_y="NEXT")
    pdf.cell(0, 7, "4-DOF Arm  |  Isaac Sim Physics Validation", align="C", new_x="LMARGIN", new_y="NEXT")
    pdf.ln(12)
    pdf.set_draw_color(*Report.ACCENT)
    pdf.set_line_width(0.8)
    cx = pdf.w / 2
    pdf.line(cx - 40, pdf.get_y(), cx + 40, pdf.get_y())
    pdf.ln(12)
    pdf.set_font("Helvetica", "", 10)
    pdf.set_text_color(*Report.BLACK)
    lines = [
        "3 arm lengths (36\", 30\", 24\")  x  2 loading configs (bare / 3 lb payload)",
        "2574 poses per configuration  |  Step environment (deployment scenario)",
        "Collision-free sweep + analytical post-filter",
        "Physics-measured torques validated against analytical gravity to < 0.5%",
    ]
    for l in lines:
        pdf.cell(0, 6, l, align="C", new_x="LMARGIN", new_y="NEXT")

    # ===================== METHOD =====================
    pdf.add_page()
    pdf.section("Method")
    pdf.body(
        "The arm is simulated in Isaac Sim with collision disabled on all arm links. "
        "This lets every joint combination be reached regardless of geometry. For each pose, "
        "the arm is teleported to the target angles, a PD controller holds position for ~0.4 s, "
        "then joint torques are measured over ~0.1 s."
    )
    pdf.body(
        "Because collision is off, many recorded poses are physically impossible (arm through "
        "the chassis, through itself, underground, etc.). A post-processing filter removes these "
        "using geometry checks. The result is two datasets per configuration: raw (every pose) "
        "and filtered (only physically realizable poses)."
    )
    pdf.body(
        "All 6 step-environment configurations achieved 2574/2574 poses converged with 0 NaN. "
        "Physics-measured torques match independent analytical gravity calculations to within 0.5%."
    )

    # ===================== ARM CONFIG =====================
    pdf.section("Arm Configuration")
    pdf.table(
        ["Parameter", "Value"],
        [
            ["Joint mass (J3, J4)", "629 g each"],
            ["Gripper mass", "500 g"],
            ["Camera mass", "98 g (Orbbec Gemini 2, on L1)"],
            ["Link tube", "0.79\" x 0.79\" pultruded CF square tube"],
            ["Loaded payload", "3 lb (1360 g) at end effector"],
        ],
        col_widths=[45, 55],
        bold_col0=False,
    )
    pdf.ln(1)
    pdf.body(
        "The camera is fixed to L1, primarily affecting J2 torque. The gripper sits at the arm tip "
        "and dominates J3/J4 loading. Adding the 3 lb payload roughly doubles peak torques across all joints."
    )

    # ===================== FILTERED RESULTS =====================
    pdf.section("Filtered Results (Valid Poses Only)")
    pdf.body(
        "Poses that pass all geometry and stability filters. These are the torques for physically "
        "realizable arm configurations on the step environment."
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
    pdf.body(
        "All torque values in Nm. Peak torques are the worst case across all valid poses -- "
        "these are what motors must be sized for. Average torques are relevant for power and thermal sizing."
    )

    # ===================== MOTOR SIZING =====================
    pdf.section("Motor Sizing")
    pdf.body(
        "Design-against numbers from the filtered step-environment data. "
        "The 36\" loaded configuration is the hardest case: J2 must hold ~19.4 Nm and J3 ~11.1 Nm. "
        "J4 requirements are modest across all configs (< 2 Nm)."
    )
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

    # ===================== LINK DEFLECTION =====================
    pdf.section("Link Deflection Under Load")
    pdf.body(
        "Computed using the 25492 square CF tube (1.250\" ID, 1.380\" OD, 0.065\" wall, "
        "E = 80.7 GPa, I = 2.42e-8 m4). "
        "Link masses estimated from the tube's linear density (0.226 lb/ft = 0.336 kg/m)."
    )

    pdf.subsection("Tip Load on L1 (Loaded Configs)")
    pdf.body(
        "Everything outboard of J2 hangs off L1: J3 joint (629 g) + L2 mass + J4 joint (629 g) "
        "+ L3 mass + end effector (gripper 500 g + payload 1360 g = 1860 g)."
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
    pdf.body("Outboard of J3: J4 joint (629 g) + L3 mass + end effector (1860 g).")
    pdf.table(
        ["Config", "Force (N)", "Length (m)", "Deflection (mm)"],
        [
            ["36\" loaded", "24.5", "0.381", "0.23"],
            ["30\" loaded", "24.4", "0.305", "0.12"],
            ["24\" loaded", "24.3", "0.305", "0.12"],
        ],
        col_widths=[22, 26, 26, 26],
    )

    pdf.subsection("Total End-Effector Sag (Loaded, Stacked)")
    pdf.table(
        ["Config", "L1", "L2", "L3", "Total Sag"],
        [
            ["36\" loaded", "0.44 mm", "0.23 mm", "~0.02 mm", "~0.7 mm"],
            ["30\" loaded", "0.30 mm", "0.12 mm", "~0.01 mm", "~0.4 mm"],
            ["24\" loaded", "0.09 mm", "0.12 mm", "~0.01 mm", "~0.2 mm"],
        ],
        col_widths=[20, 20, 20, 20, 20],
    )
    pdf.body("Total sag is under 1 mm even in the worst case (36\" loaded, arm fully extended). The CF tube is more than stiff enough.")

    # ===================== STABILITY =====================
    pdf.section("Chassis Stability")
    pdf.body(
        "Static tipping analysis: the combined center of gravity of the chassis (20.4 kg) plus all "
        "arm masses is projected onto the ground plane and checked against the wheel support polygon. "
        "J1 is swept from 0 to 345 deg in 15 deg steps; the worst-case J1 angle is reported."
    )
    pdf.note(
        "This assumes a level chassis. The 4.5\" step creates a permanent tilt that shifts the CG "
        "projection and reduces stability margins on the downhill side. Step tilt correction is a follow-up."
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
        "The rejected poses are all fully-extended configurations with the arm reaching sideways "
        "(J1 ~ 90 deg), perpendicular to the long axis of the chassis. The chassis is narrower side-to-side "
        "(~9.4\") than front-to-back (~15.4\"), so sideways reach is the worst case."
    )
    pdf.body(
        "The 24\" arm is unconditionally stable. The 30\" arm has a small number of marginal poses when "
        "loaded. The 36\" loaded arm has ~10% of poses rejected for tipping, all at full extension."
    )

    # ===================== FILTER DETAILS =====================
    pdf.section("Filtering Details")
    pdf.body(
        "A pose is rejected if any of the following checks fail:"
    )
    filters = [
        ("Chassis collision", "10 sample points along each link. If any point falls inside the chassis bounding box (10 mm clearance), the pose is rejected."),
        ("Self-collision (L3 vs L1)", "Minimum distance between L3 and L1 segments, sampled at 12 points each. Rejected if closer than 15 mm."),
        ("Self-collision (L3 vs J1)", "8 sample points along L3 checked against the J1 base volume. Rejected if any point enters the base."),
        ("Floor collision", "Any joint position or link midpoint below the floor plane (10 mm + tube half-width margin)."),
        ("Tipping", "CG projection outside the wheel support polygon at any J1 angle."),
    ]
    for name, desc in filters:
        pdf.set_font("Helvetica", "B", 9.5)
        pdf.set_text_color(*Report.BLACK)
        pdf.cell(pdf.get_string_width(name + "  "), 5, name + "  ")
        pdf.set_font("Helvetica", "", 9.5)
        # Use remaining width for description
        pdf.multi_cell(0, 5, desc)
        pdf.ln(1)

    pdf.subsection("Rejection Breakdown")
    pdf.table(
        ["Arm", "Loading", "Total", "Valid", "Chassis", "Self L3-L1", "Self L3-J1", "Floor", "Tip"],
        [
            ["24\"", "bare",   "2574", "1845 (71.7%)", "487", "100", "0",   "142", "0"],
            ["24\"", "loaded", "2574", "1845 (71.7%)", "487", "100", "0",   "142", "0"],
            ["30\"", "bare",   "2574", "1890 (73.4%)", "380", "100", "0",   "204", "0"],
            ["30\"", "loaded", "2574", "1867 (72.5%)", "380", "100", "0",   "204", "23"],
            ["36\"", "bare",   "2574", "1880 (73.0%)", "353",  "74", "4",   "263", "0"],
            ["36\"", "loaded", "2574", "1694 (65.8%)", "353",  "74", "4",   "263", "186"],
        ],
        col_widths=[10, 14, 12, 22, 14, 16, 16, 12, 10],
        bold_col0=False,
    )
    pdf.body(
        "Loading doesn't change geometry rejections (chassis, self-collision, floor) because the "
        "geometry is the same. It only affects tipping, because the 3 lb payload shifts the CG further outboard."
    )

    # ===================== RAW DATA =====================
    pdf.section("Raw Torque Data (Unfiltered)")
    pdf.body(
        "For reference: torques before filtering. Includes poses where the arm passes through "
        "the chassis, through itself, or underground. Step environment only."
    )
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

    # ===================== FLAT SUPPLEMENTARY =====================
    pdf.section("Supplementary: Flat Environment Data")
    pdf.note(
        "This data is NOT reliable for motor sizing. On the flat surface, the chassis is held in place "
        "only by wheel friction. When the arm teleports to each pose, the chassis slides and the measured "
        "torques include dynamic forces from the moving base -- not just gravity. Physics-vs-analytical "
        "agreement is off by 100-200%, compared to < 0.5% on the step. Use step numbers for all design decisions."
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
    pdf.body("All values in Nm. Filtered, flat environment.")

    pdf.output(PDF_PATH)
    print(f"PDF written to {PDF_PATH}")


if __name__ == "__main__":
    build()
