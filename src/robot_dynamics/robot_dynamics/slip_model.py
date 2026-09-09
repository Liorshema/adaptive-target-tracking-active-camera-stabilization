class SlipModel:
    def __init__(self, slip_ratio):
        if not 0.0 <= slip_ratio <= 1.0:
            raise ValueError("slip_ratio must be between 0 and 1")

        self.slip_ratio = slip_ratio

    def apply_slip(self, ideal_force):
        """
        Reduce ideal traction force according to slip ratio.

        slip_ratio = 0.0 -> no slip
        slip_ratio = 0.3 -> 30% force loss
        slip_ratio = 1.0 -> complete loss of traction
        """

        effective_force = (
            1.0 - self.slip_ratio
        ) * ideal_force

        return effective_force