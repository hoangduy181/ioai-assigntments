import numpy as np
from state import State

#
#                       _oo0oo_
#                      o8888888o
#                      88" . "88
#                      (| -_- |)
#                      0\  =  /0
#                    ___/`---'\___
#                  .' \\|     |// '.
#                 / \\|||  :  |||// \
#                / _||||| -:- |||||- \
#               |   | \\\  -  /// |   |
#               | \_|  ''\---/''  |_/ |
#               \  .-\__  '-'  ___/-. /
#             ___'. .'  /--.--\  `. .'___
#          ."" '<  `.___\_<|>_/___.' >' "".
#         | | :  `- \`.;`\ _ /`;.`/ - ` : | |
#         \  \ `_.   \_ __\ /__ _/   .-` /  /
#     =====`-.____`.___ \_____/___.-`___.-'=====
#                       `=---='
def check_local(successor: State, undoneBoard):
    
    local_points = 0
    for i in undoneBoard:
        local_board = np.copy(successor.blocks[i])
        local_cells = local_board.reshape((1,9))

        local_row = np.sum(local_board, 1)
        local_col = np.sum(local_board, 0)

        local_tl_diag = local_board.trace()
        local_tr_diag = local_board[::-1].trace()

        local_sequence = sum(sq for sq in local_col if sq == 2 or sq ==-2) + sum(sq for sq in local_row if sq == 2 or sq ==-2)
        local_sequence+= local_tl_diag if (local_tl_diag == 2 or local_tl_diag ==-2) else 0
        local_sequence+= local_tr_diag if (local_tr_diag == 2 or local_tr_diag ==-2) else 0

        #center and sequence:
        local_points += local_cells[0][4]*3 + local_sequence    


    return local_points

def check_global(successor: State):

    global_cells = np.copy(successor.global_cells)  
    global_board = global_cells.reshape(3,3)

    game_result = successor.game_result(global_board)
    #game is over: someone wins
    if game_result != None:
        return 9999*game_result
    #game is over: draw:
    
    undoneBoard = []

    #game is not over

    #undone
    i = np.where(global_board == 0)
    undoneBoard = [index[0]*3 + index[1]
                for index in list(zip(i[0], i[1]))]
    #print("undone: ",undoneBoard)
    #sequence: 2 block in-a-row(col,diag,..) without an opposite block in the rest position

    global_row = np.sum(global_board, 1)
    global_col = np.sum(global_board, 0)

    global_tl_diag = global_board.trace()
    global_tr_diag = global_board[::-1].trace()

    global_sequence = sum(sq for sq in global_col if sq == 2 or sq ==-2) + sum(sq for sq in global_row if sq == 2 or sq ==-2)
    global_sequence+= global_tl_diag if (global_tl_diag == 2 or global_tl_diag ==-2) else 0
    global_sequence+= global_tr_diag if (global_tr_diag == 2 or global_tr_diag ==-2) else 0

    #center and corners:

    global_points = 0
    global_sum = np.sum(global_board) * 5
    center = global_cells[4] * 10 
    corner = sum(global_cells[i] for i in [0,2,6,8]) * 3
    sequence = global_sequence*2

    global_points = global_sum + center + corner + sequence

    block_points = check_local(successor, undoneBoard)

    free_move_points = 2*successor.player_to_move if successor.free_move else 0

    return global_points + block_points + free_move_points

def heuristic(successor):
    return check_global(successor)

#minimax with alpha-beta pruning
def alphabeta(state: State, depth, alpha, beta, player, my_player, _depth):              
    #fail-hard variation
    if depth == _depth:
        value = heuristic(state)
        #print("child value: ", value, "at depth: ", depth)
        return value, None

    valid_moves = state.get_valid_moves
    if len(valid_moves) == 0:
        return heuristic(state), None

    if player == 1: #maxplayer
        value = -100000
        best_move = None

        #tim move tot nhat
        for move in valid_moves:
            temp_state = State(state)
            temp_state.free_move = state.free_move
            temp_state.act_move(move)

            new_value, temp_move = alphabeta(temp_state, depth + 1, alpha, beta, -1, my_player, _depth)
            
            if value < new_value:
                value = new_value
                best_move = move
            if value >= beta:
                break
            alpha = max(alpha, value)

        return value, best_move
    else:
        value = 100000
        best_move = None

        for move in valid_moves:
            temp_state = State(state)
            temp_state.free_move = state.free_move
            temp_state.act_move(move)

            new_value, temp_move = alphabeta(temp_state, depth + 1, alpha, beta, 1, my_player, _depth)  
            
            if value > new_value:
                value = new_value
                best_move = move

            if value <= alpha:
                break
            beta = min(beta, value)
        return value, best_move

def select_move(cur_state, remain_time):
    my_player = cur_state.player_to_move
    valid_moves = cur_state.get_valid_moves
    if len(valid_moves) != 0:
        value, move = alphabeta(cur_state, 0, -100000, +100000, my_player, my_player, 4)
        return move
    return None