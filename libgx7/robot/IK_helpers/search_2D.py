import numpy as np
from scipy.optimize import minimize

def search_2D(fun, min1, max1, min2, max2, N, show_plot=False):
    N_max_minima = 100
    MIN_THRESH = 1e-1

    x1_vec = np.full(N_max_minima, np.nan)
    x2_vec = np.full(N_max_minima, np.nan)
    soln_num_vec = np.full(N_max_minima, np.nan)

    # Sample the search space
    x1_search_vec = np.linspace(min1, max1, N)
    x2_search_vec = np.linspace(min2, max2, N)
    x1_mesh, x2_mesh = np.meshgrid(x1_search_vec, x2_search_vec)

    test_output = fun(min1, min2)
    N_branches = len(test_output)

    e_mesh = np.full((N, N, N_branches), np.nan)
    for i in range(N):
        for j in range(N):
            x1_ij = x1_mesh[i, j]
            x2_ij = x2_mesh[i, j]
            f_ij = fun(x1_ij, x2_ij)
            e_mesh[i, j, :] = f_ij

    # Find approximate minima
    lt_thresh_mat = e_mesh < MIN_THRESH
    e_mesh_iter = np.copy(e_mesh)
    e_mesh_iter[~lt_thresh_mat] = np.nan

    for i_minimum in range(N_max_minima):
        if np.all(np.isnan(e_mesh_iter)):
            break
        # Find smallest value
        idx_flat = np.nanargmin(e_mesh_iter)
        idx_min_1, idx_min_2, idx_min_3 = np.unravel_index(idx_flat, e_mesh_iter.shape)
        x_min_i = e_mesh_iter[idx_min_1, idx_min_2, idx_min_3]

        if np.isnan(x_min_i) or x_min_i > MIN_THRESH:
            break

        x1_vec[i_minimum] = x1_mesh[idx_min_1, idx_min_2]
        x2_vec[i_minimum] = x2_mesh[idx_min_1, idx_min_2]
        soln_num_vec[i_minimum] = idx_min_3

        # Remove all neighbors below threshold
        neighbors_idx = find_blob(idx_min_1, idx_min_2, lt_thresh_mat[:, :, idx_min_3])
        for ni in neighbors_idx:
            i_n, j_n = ni
            lt_thresh_mat[i_n, j_n, idx_min_3] = False
            e_mesh_iter[i_n, j_n, idx_min_3] = np.nan

    # Remove unused entries
    valid = ~np.isnan(x1_vec)
    x1_vec = x1_vec[valid]
    x2_vec = x2_vec[valid]
    soln_num_vec = soln_num_vec[valid].astype(int)

    # Refine minima using local optimization
    for i in range(len(x1_vec)):
        res = minimize(lambda ip: select_soln(fun, ip[0], ip[1], soln_num_vec[i]),
                       x0=[x1_vec[i], x2_vec[i]], method='Nelder-Mead')
        x1_vec[i], x2_vec[i] = res.x

    if show_plot:
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        for i in range(N_branches):
            ax.plot_surface(x1_mesh, x2_mesh, e_mesh[:, :, i], alpha=0.5)
        ax.scatter(x1_vec, x2_vec, np.zeros_like(x1_vec), color='r', marker='x')
        plt.show()

    return x1_vec, x2_vec, soln_num_vec


def find_blob(x1, x2, bool_mat):
    sz = bool_mat.shape
    Q = [(x1, x2)]
    neighbors_idx = set()

    while Q:
        starting_x1, starting_x2 = Q.pop()
        # Wrap for torus (angles)
        starting_x1 = starting_x1 % sz[0]
        starting_x2 = starting_x2 % sz[1]

        if (starting_x1, starting_x2) in neighbors_idx:
            continue

        if bool_mat[starting_x1, starting_x2]:
            neighbors_idx.add((starting_x1, starting_x2))
            # Add neighbors
            Q.extend([
                ((starting_x1 + 1) % sz[0], starting_x2),
                ((starting_x1 - 1) % sz[0], starting_x2),
                (starting_x1, (starting_x2 + 1) % sz[1]),
                (starting_x1, (starting_x2 - 1) % sz[1])
            ])

    return list(neighbors_idx)


def select_soln(fun, x1, x2, soln_num):
    v = fun(x1, x2)
    return v[soln_num]
