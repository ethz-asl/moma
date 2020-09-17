from trac_ik_python.trac_ik import IK
# import trac_ik_python

def main():
    with open("data/ridgeback_panda_hand.urdf") as f:
        if f.mode == 'r':
            urdf_string = f.read()

    ik_solver = IK("panda_link0", "panda_link8", urdf_string=urdf_string)

    seed_state = [0.0] * ik_solver.number_of_joints

    sol = ik_solver.get_ik(seed_state, 0.47, 0.0, 0.1, 0.9239557, -0.38249949,  0.0,  0.0)
    print(sol)

if __name__ == '__main__':
    main()
    # print(trac_ik_python.__file__)
