import numpy as np
from scipy.optimize import root_scalar
import matplotlib.pyplot as plt


def search_1D(fun, x1, x2, N, show_plot=False):
    """
    Search for zeros of a vector-valued function over a 1D interval.

    Parameters
    ----------
    fun : callable
        Function that returns a vector given a scalar input.
    x1, x2 : float
        Search interval.
    N : int
        Number of initial samples.
    show_plot : bool
        Whether to plot results.

    Returns
    -------
    x_vec : np.ndarray
        Locations of zeros.
    soln_num_vec : np.ndarray
        Index of the function component that has the zero.
    """

    # Sample the search space
    x_sample_vec = np.linspace(x1, x2, N)
    e_1 = fun(x_sample_vec[0])
    e_mat = np.full((len(e_1), N), np.nan)
    e_mat[:, 0] = e_1
    for i in range(1, N):
        e_mat[:, i] = fun(x_sample_vec[i])

    # Find zero crossings
    CROSS_THRESH = 0.1
    zero_cross_direction = (np.diff(e_mat < 0, axis=1) != 0) & \
                           (np.abs(e_mat[:, 1:]) < CROSS_THRESH) & \
                           (np.abs(e_mat[:, :-1]) < CROSS_THRESH)

    has_zero_cross = np.sum(zero_cross_direction, axis=0) > 0
    crossings_left = x_sample_vec[has_zero_cross]
    crossings_right = x_sample_vec[np.concatenate(([False], has_zero_cross))]

    crossing_soln_nums = zero_cross_direction[:, has_zero_cross]
    n_zeros = np.sum(crossing_soln_nums)

    # Iterate on each bracket
    x_vec = []
    soln_num_vec = []
    for i in range(len(crossings_left)):
        soln_nums = np.where(crossing_soln_nums[:, i])[0]
        for soln_num in soln_nums:
            bracket = [crossings_left[i], crossings_right[i]]
            try:
                res = root_scalar(lambda x: select_soln(fun(x), soln_num),
                                  bracket=bracket, method='brentq', xtol=1e-5)
                if res.converged:
                    x_vec.append(res.root)
                    soln_num_vec.append(soln_num)
            except ValueError:
                # Root not bracketed, skip
                continue

    x_vec = np.array(x_vec)
    soln_num_vec = np.array(soln_num_vec)

    # Plot results
    if show_plot:
        plt.figure()
        for i in range(e_mat.shape[0]):
            plt.plot(x_sample_vec, e_mat[i, :], '.')
        plt.axhline(0, color='k')
        if len(x_vec) > 0:
            for xc in x_vec:
                plt.axvline(xc, color='r')
        plt.show()

    return x_vec, soln_num_vec


def select_soln(x_arr, soln_num):
    return x_arr[soln_num]
