import matplotlib.pyplot as plt
import simulation
from scenario import Scenario


def main():
    scenario = Scenario(
        name="bigger",
        a1=0.15,
        a2=0.2,
        a3=0.2,
        a4=0.15,
        a5=0.05,
        x1=0,
        y1=0,
        ratio=1,
        Tmax=0.38,  # dual motors
        w=0.30,
        h=0.15,
        xcenter=-0.025,
        ycenter=0.2,
        xmin=-0.25,
        xmax=0.2,
        ymin=-0.05,
        ymax=0.30,
    )

    small_scenario = Scenario(
        name="small",
        a1=0.065,
        a2=0.1,
        a3=0.1,
        a4=0.065,
        a5=0.05,
        x1=0,
        y1=0,
        ratio=1,
        Tmax=0.19,
        w=0.15,
        h=0.075,
        xcenter=-0.025,
        ycenter=0.08,
        xmin=-0.1,
        xmax=0.15,
        ymin=-0.3,
        ymax=0.05,
    )

    original_scenario = Scenario(
        name="original",
        a1=0.25,
        a2=0.25,
        a3=0.25,
        a4=0.25,
        a5=0.1,
        x1=0,
        y1=0,
        ratio=0.07303 / (0.0131 / 2),  # capstan
        Tmax=0.129,
        w=0.2794,
        h=0.2159,
        xcenter=-0.05,
        ycenter=0.34,
        xmin=-0.25,
        xmax=0.35,
        ymin=-0.45,
        ymax=0.03,
    )
    simulation.envelope(scenario)
    simulation.reach(scenario)
    simulation.examples(scenario)
    simulation.interior(scenario)
    plt.show()


if __name__ == "__main__":
    main()
