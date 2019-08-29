"""
The mergeMap class contains the strategy to merge maps.
It is based on approach of using complete representation of the map and then converting everything in cartesian plane and merging those maps.
Used previously in the project, not integrated with map.py
__author__ = '{Mayank Yadav}'
__project__='{AAIP Project SS19}'
"""


class mergeMap(object):

    def mapping(self, map1, map2, goal):

        # change_agent_val(map1,5)
        # change_agent_val(map2,5)

        point1 = np.where(map1 == goal)
        point2 = np.where(map2 == goal)
        origin_map1 = (point1[0][0], point1[1][0])  # choosing row and col
        origin_map2 = (point2[0][0], point2[1][0])  # choosing row and col
        # for maximum advaantage, max value of each row and col will be origin of final map.
        # hence
        final_map_origin = (max(point1[0][0], point2[0][0]), max(point1[1][0], point2[1][0]))

        new_dist_map1 = np.empty((map1.shape[0], map1.shape[1]), dtype=object)
        new_dist_map2 = np.empty((map2.shape[0], map2.shape[1]), dtype=object)

        # distance map for both maps for both maps

        for i in range(0, map1.shape[0]):
            for j in range(0, map1.shape[1]):
                new_dist_map1[i][j] = (i - origin_map1[0], j - origin_map1[1])

        for i in range(0, map2.shape[0]):
            for j in range(0, map2.shape[1]):
                new_dist_map2[i][j] = (i - origin_map2[0], j - origin_map2[1])

        row_up_mapper = 0

        col_left_mapper = 0

        row_to_up = max(np.min(point1[0]), np.min(point2[0]))

        if (np.min(point1[0]) == row_to_up):
            row_up_mapper = 1
        else:
            row_up_mapper = 2

        row_to_down = max(map1.shape[0] - np.max(point1[0]), map2.shape[0] - np.max(point2[0]))

        size_row = np.max(point1[0]) - np.min(point1[0])

        col_to_left = max(np.min(point1[1]), np.min(point2[1]))

        if (np.min(point1[1]) == col_to_left):
            col_left_mapper = 1
        else:
            col_left_mapper = 2

        col_to_right = max(map1.shape[1] - np.max(point1[1]), map2.shape[1] - np.max(point2[1]))
        size_col = np.max(point1[1]) - np.min(point1[1])
        total_col_size = col_to_left + col_to_right + size_col
        total_row_size = row_to_up + row_to_down + size_row

        merge_map = np.full((total_row_size, total_col_size), -1)
        merge_map_distance = np.empty((total_row_size, total_col_size), dtype=object)
        for i in range(0, total_row_size):
            for j in range(0, total_col_size):
                merge_map_distance[i][j] = (i - final_map_origin[0], j - final_map_origin[1])

        print('FINAL MAP ORIGIN VALUES')

        for i in range(0, total_row_size):
            for j in range(0, total_col_size):
                search_tuple = merge_map_distance[i][j]
                value_map_1 = 500
                value_map_2 = 1000
                for k in range(0, map1.shape[0]):
                    for l in range(0, map1.shape[1]):
                        if (search_tuple == new_dist_map1[k][l]):
                            value_map_1 = map1[k][l]

                for m in range(0, map2.shape[0]):
                    for n in range(0, map2.shape[1]):
                        if (search_tuple == new_dist_map2[m][n]):
                            value_map_2 = map2[m][n]

                if (value_map_1 == 500 and value_map_2 == 1000):
                    merge_map[i][j] = -1  # not found in any one of them
                elif (value_map_1 == 500 and value_map_2 != 1000):
                    merge_map[i][j] = value_map_2
                elif (value_map_1 != 500 and value_map_2 == 1000):
                    merge_map[i][j] = value_map_1
                else:
                    if (value_map_1 == value_map_2):
                        merge_map[i][j] = value_map_2  # any one of them
                    else:
                        if (value_map_1 == -1 and value_map_2 == 0):
                            merge_map[i][j] = value_map_2
                        elif (value_map_1 == 0 and value_map_2 == -1):
                            merge_map[i][j] = value_map_1
                        elif (value_map_2 is obstacle or value_map_1 is obstacle):
                            merge_map[i][j] = obstacle
                        elif (value_map_1 is dispenser or value_map_2 is dispenser):
                            merge_map[i][j] = dispenser

        return merge_map
