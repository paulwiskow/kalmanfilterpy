from rocketpy import Environment, SolidMotor, Rocket, Flight
import datetime
from math import pi, pow

class Sim:

    def simulate(self):
        # %% md
        # Global Parameters
        # %%
        # Rocket Dimension & Mass Properties
        #       You can get the Mass and CG values from the OpenRocket model
        #       click Tools > Component analysis
        #       Note that the mass should be without the motor

        #       To get the inertias you can go to the simulation tab and export the
        #       longitudinal and lateral inertias as a function of time
        rocket_diameter_mm = 6.0 * 25.43
        rocket_mass_oz = 646
        rocket_mass_kg = (rocket_mass_oz / 16) / 2.20462
        rocket_inertia_xx_yy_kgpm2 = 29.871
        rocket_inertia_zz_kgpm2 = 0.128
        rocket_center_of_mass_in = 63.027
        rocket_coordinate_system_orientation = "nose_to_tail"

        # Fin Parameters
        shear_modulus_psi = 20_000
        semi_span_in = 6.25
        root_chord_in = 15.0
        tip_chord_in = 7.0
        thickness_in = 3 / 8
        fin_area_in2 = 0.5 * (root_chord_in + tip_chord_in) * semi_span_in
        aspect_ratio = semi_span_in ** 2 / fin_area_in2
        chord_ratio = tip_chord_in / root_chord_in

        def temp_at_alt_F(alt_ft):
            return 59 - 0.00356 * alt_ft

        def pres_lbspin2(temp_F):
            return (2116 / 144) * ((temp_F + 459.7) / 518.6) ** 5.256

        def sound_speed_ftps(temp_F):
            return (1.4 * 1716.59 * (temp_F + 460)) ** 2

        def fin_flutter_velocity_ftpsec(sound_speed_ftps, shear_modulus_psi, aspect_ratio, pres_lbspin2, chord_ratio,
                                        thickness_in, root_chord_in):
            return sound_speed_ftps * \
                ((shear_modulus_psi / (1.337 * aspect_ratio ** 3 * pres_lbspin2 * (chord_ratio + 1))) \
                 / (2 * (aspect_ratio + 2) * (thickness_in / root_chord_in))) ** (1 / 2)

        # Rocket Aerodynamic Properties (Power On/Off Drag Can be Downloaded from RASAero)
        rocket_power_off_drag_file_location = ""
        rocket_power_on_drag_file_location = ""

        #       alternatively, you can just call this function, which returns the
        #       drag coeff as a function of mach
        def rocket_drag(mach):
            return 0.54

        # Data from ThrustCurve for M2000R
        motor_thrust_file = "data/AeroTech_M2000R.eng"
        motor_diameter_mm = 98
        motor_length_mm = 732
        motor_total_weight_g = 8_987
        motor_prop_weight_g = 5_368

        # Initial Conditions
        initial_height_m = 1400.0

        # Initialize Environment Object
        #   This location roughly corresponds to the Spaceport America Launch Site
        env = Environment(latitude=32.990254, longitude=-106.974998, elevation=initial_height_m)

        # Example of using predicted winds
        tomorrow = datetime.date.today() + datetime.timedelta(days=1)

        env.set_date((tomorrow.year, tomorrow.month, tomorrow.day, 12))  # Hour given in UTC time

        # The line below takes ~20s to run because it's pulling
        # a wind forecast from a server/website
        # env.set_atmospheric_model(type="Forecast", file="GFS")

        # env.info()
        # %% md
        # Defining our Motor
        # We
        # need
        # info
        # about
        # the
        # grain
        # of
        # our motor, which do not have / haven't calculated yet. These numbers are best guesses at the moment.
        #
        # https: // www.thrustcurve.org / motors / AeroTech / M2000R /
        #
        # https: // www.apogeerockets.com / Rocket_Motors / Rouse - Tech_Casings / 98
        # mm_Casings / RMS - 98_10240
        # _Casing
        # %%
        # Dry Mass (mass of motor without propellant)
        motor_dry_mass_kg = (motor_total_weight_g - motor_prop_weight_g) / 1000

        # Dry Inertia (inertia of motor without propellant)
        #       Essentially just the inertia of the motor case
        #       which is assumed to be cylindrical shell
        motor_dry_inertia_xx_yy = motor_dry_mass_kg * \
                                  ((1 / 2) * (motor_diameter_mm / 2000) ** 2 + (1 / 12) * (motor_length_mm / 1000) ** 2)
        motor_dry_inertia_zz = motor_dry_mass_kg * (motor_diameter_mm / 2000) ** 2.0
        # print(motor_dry_inertia_xx_yy)  # looks reasonable
        # print(motor_dry_inertia_zz)

        # Grain Info
        motor_grain_count = 4
        motor_grain_inner_diameter_mm = 15 * 2  # guestimate
        motor_grain_volume_m3 = (pi / 4) * (motor_length_mm / 1000) * \
                                ((motor_diameter_mm / 1000) ** 2 - (motor_grain_inner_diameter_mm / 1000) ** 2)
        motor_grain_density_kgpm3 = (motor_prop_weight_g / 1000) / motor_grain_volume_m3
        # print(motor_grain_volume_m3) # reasonable, can check against the motor.info() output
        # print(motor_grain_density_kgpm3)

        # These require SI units (kg, m)
        #       Nozzle Radius,
        #       Assumes center of dry and wet mass are the same (half of motor length)
        AeroTechM2000R = SolidMotor(
            thrust_source=motor_thrust_file,
            dry_mass=motor_dry_mass_kg,
            dry_inertia=(motor_dry_inertia_xx_yy, motor_dry_inertia_xx_yy, motor_dry_inertia_zz),
            nozzle_radius=33 / 1000,
            grain_number=motor_grain_count,
            grain_density=motor_grain_density_kgpm3,
            grain_outer_radius=(motor_diameter_mm / 2000),
            grain_initial_inner_radius=(motor_grain_inner_diameter_mm / 2000),
            grain_initial_height=(motor_length_mm / 1000) / motor_grain_count,
            grain_separation=0 / 1000,
            grains_center_of_mass_position=(motor_length_mm / 2000),
            center_of_dry_mass_position=(motor_length_mm / 2000),
            nozzle_position=0,
            burn_time=5.0,
            throat_radius=11 / 1000,
            coordinate_system_orientation="nozzle_to_combustion_chamber",
        )

        # AeroTechM2000R.info()
        # AeroTechM2000R.all_info()
        # %% md
        # Defining our Rocket

        # %%
        Sabre_1 = Rocket(
            radius=rocket_diameter_mm / 2000,
            mass=rocket_mass_kg,
            inertia=(rocket_inertia_xx_yy_kgpm2, rocket_inertia_xx_yy_kgpm2, rocket_inertia_zz_kgpm2),
            power_off_drag=rocket_drag(float),
            power_on_drag=rocket_drag(float),
            center_of_mass_without_motor=(rocket_center_of_mass_in * 0.0254),
            coordinate_system_orientation=rocket_coordinate_system_orientation,
        )

        Sabre_1.add_motor(AeroTechM2000R, position=112.5 * 0.0254)

        rail_buttons = Sabre_1.set_rail_buttons(
            upper_button_position=65 * 0.0254,
            lower_button_position=90 * 0.0254,
            angular_position=45,
        )

        nose_cone = Sabre_1.add_nose(
            length=23 * 0.0254,
            kind="von karman",
            position=0.0
        )

        fin_set = Sabre_1.add_trapezoidal_fins(
            n=4,
            root_chord=(15 * 0.0254),
            tip_chord=(7 * 0.0254),
            span=(6.25 * 0.0254),
            position=(95 * 0.0254),
            cant_angle=0.0,
            sweep_angle=64.1,
            airfoil=None,
        )

        # Sabre_1.info()
        # Sabre_1.plots.static_margin()
        # %%
        nominal_flight = Flight(
            rocket=Sabre_1,
            environment=env,
            rail_length=17 * 0.3048,
            inclination=84,
            heading=0,
            terminate_on_apogee=True
        )

        # print(
        #     f"Nominal Apogee: {nominal_flight.apogee - initial_height_m} m, {(nominal_flight.apogee - initial_height_m) * 3.28084} ft")
        # nominal_flight.z.plot(0, nominal_flight.apogee_time)
        # nominal_flight.all_info()
        nominal_flight.export_data("test.csv", "x", "vx", "y", "vy", "z", "vz", time_step=.1)