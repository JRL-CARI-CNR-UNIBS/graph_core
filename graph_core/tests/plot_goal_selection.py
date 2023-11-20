import rospy
import matplotlib.pyplot as plt

if __name__ == '__main__':

    cost_baseline = rospy.get_param("baseline/cost")
    cost_egreedy = rospy.get_param("egreedy/cost")

    arm_baseline = rospy.get_param("baseline/arm")
    arm_egreedy = rospy.get_param("egreedy/arm")

    bins_max = max(arm_baseline)
    bins=range(0,bins_max+1)
    print(bins)

    fig, axes = plt.subplots(nrows=1, ncols=2, figsize=(25, 5))

    axes[0].plot(cost_baseline,label="baseline")
    axes[0].plot(cost_egreedy,label="egreedy")
    axes[0].legend()
    axes[0].set_title("cost")

    axes[1].hist(arm_baseline,bins=bins,label="baseline")
    axes[1].hist(arm_egreedy,bins=bins,label="egreedy")
    axes[1].legend()
    axes[1].set_title("arm")

    print(arm_baseline)
    print(arm_egreedy)
    
    plt.show()