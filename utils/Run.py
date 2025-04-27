import builtins
import threading
import contextlib
from utils.index_deal import *
import colorama
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from functools import wraps

thread_local = threading.local()


class Procedure:
    def __init__(self):
        self.cur_num = 0
        self.cur_index = 1
        self.temp = 1
        self.is_first = True
        self.index_list = []
        self.index_list_dic = {}
        self.tasks = None
        self.index_len = 0
        self.count = 0
        self.original_print = builtins.print
        self.cur_main_color = None
        self.judge = None
        self.judge_num = 0
        self.temp_judge_num = 0
        self.is_judge = False
        self.run_info = {}
        self.cur_judge_key = None
        self.thread_colors_dic = {
            "ThreadPoolExecutor-0_1": "white",
            "ThreadPoolExecutor-0_2": "magenta",
            "ThreadPoolExecutor-0_3": "blue",
            "ThreadPoolExecutor-0_4": "green",
            "ThreadPoolExecutor-0_5": "yellow",
            "ThreadPoolExecutor-0_6": "cyan",
        }
        self.return_result = None

    def create_tasks(self, tasks):
        self.tasks = tasks
        self.analysis_tasks(tasks)
        self.index_len = len(self.index_list)

    def color_print_(self, text, *args, color="green", is_bright=False, **kwargs):
        if color == "red":
            color = colorama.Fore.RED
        elif color == "green":
            color = colorama.Fore.GREEN
        elif color == "yellow":
            color = colorama.Fore.YELLOW
        elif color == "blue":
            color = colorama.Fore.BLUE
        elif color == "magenta":
            color = colorama.Fore.MAGENTA
        elif color == "cyan":
            color = colorama.Fore.CYAN
        elif color == "white":
            color = colorama.Fore.WHITE
        elif color == "black":
            color = colorama.Fore.BLACK
        else:
            color = colorama.Fore.RESET

        if is_bright:
            color += colorama.Style.BRIGHT

        colored_args = [color + str(arg) for arg in args]
        self.original_print(color+text, *colored_args, colorama.Fore.RESET)
        # self.original_print()
        return True

    def color_print(self, text, *args, color="red", is_bright=False, is_choose_color=False, **kwargs):
        num = self.cur_num
        current_thread = threading.current_thread()
        text = str(text)
        if current_thread.name == "MainThread":
            if not is_choose_color:
                if num == 1:
                    color = "white"
                elif num == 2:
                    color = "blue"
                elif num == 3:
                    color = "yellow"
                elif num == 4:
                    color = "green"
                elif num == 5:
                    color = "cyan"
                elif num == 6:
                    color = "magenta"
                else:
                    color = "red"
            self.cur_main_color = color
        else:
            info = thread_local.info
            group = thread_local.group
            if current_thread.name in self.thread_colors_dic:
                color = self.thread_colors_dic[current_thread.name]
            else:
                color = "red"

            text = f"{group} {info}:  " + text

        if num == 1:
            tab = ""
        else:
            tab = "     " * (num - 1)

        self.color_print_(tab + text, *args, color=color, is_bright=is_bright, **kwargs)
        return True

    def judge_(self, val):  # 判断值（流程路径选择） 可自行添加
        if val:
            self.judge = 0
        elif not val:
            self.judge = 1

    def analysis_tasks_(self, tasks, index=0, num=1):
        if isinstance(tasks, dict):  # 是否有小步骤
            while True:
                time.sleep(0.1)
                if index < len(tasks):
                    for i in range(len(tasks) - index):

                        if num < self.temp:  # 检查是否返回了上一层的嵌套
                            self.temp = num
                            self.cur_index = index_by_reduce_num(self.cur_index, num)
                        elif num > self.temp:  # 检查是否进入了下一层的嵌套
                            self.cur_index = index_by_add_num(self.cur_index)
                            self.temp = num

                        self.cur_num = num
                        goal = list(tasks.keys())[index]
                        task = tasks[goal]
                        # self.color_print_(f"{self.cur_index}  步骤 {goal}, 嵌套当前层数 {num}", is_bright=True)
                        if self.cur_index not in self.index_list:
                            self.index_list.append(self.cur_index)
                        result = self.analysis_tasks_(task, index=0, num=num + 1)

                        if result:
                            # 嵌套完成了，
                            index += 1
                            self.cur_index = add_index(self.cur_index)
                        elif result is False:
                            index += 1
                            self.cur_index = add_index(self.cur_index)

                        if index == len(tasks):
                            # 代表完成这一步的所有小步骤
                            return True

        elif isinstance(tasks, list):
            # task = tasks[1]
            self.is_judge = True
            list_len = len(tasks)
            temp_index = self.cur_index
            temp_num = self.cur_num
            temp = self.temp
            temp_index_list = self.index_list
            dic_key = list(self.index_list_dic.keys())
            print(tasks)
            for i, task in enumerate(tasks):
                self.cur_index = temp_index
                self.cur_num = temp_num
                self.temp = temp
                self.index_list = temp_index_list.copy()
                result = self.analysis_tasks_(task, index=0, num=num)
                if self.judge_num != 0:
                    for selection in range(list_len):
                        # if selection + 1 != list_len:
                        temp_index_list = self.index_list.copy()
                        other_index = self.run_info[str(self.judge_num * 2 - 1)]["cur_index"]   # 上一次判断在哪个步骤
                        index_index = temp_index_list.index(other_index)
                        goal_index = None
                        for index_ in temp_index_list[index_index:]:
                            if other_index in index_:
                                continue
                            else:
                                goal_index = index_
                                break
                        if goal_index is not None:
                            goal_index_list = temp_index_list[temp_index_list.index(goal_index):]

                        save_index = judge_register(self.judge_num, i, dic_key[-(selection + 1)])
                        self.index_list_dic[save_index] = self.index_list_dic[save_index[:-2]] + goal_index_list
                        self.run_info[save_index] = {"cur_index": temp_index, "cur_num": self.cur_num, "temp": self.temp, "index_list": self.index_list.copy()}
                else:
                    self.index_list_dic[str(i)] = self.index_list.copy()
                    self.run_info[str(i)] = {"cur_index": temp_index,
                                             "cur_num": self.cur_num, "temp": self.temp,
                                             "index_list": self.index_list.copy()}
            self.judge_num += 1
            if result:
                # 嵌套完成了，
                index += 1
                self.cur_index = add_index(self.cur_index)

            elif result is False:
                index += 1
                # if index != len(tasks):
                self.cur_index = add_index(self.cur_index)

            if index == 1:
                # 代表完成这一步的所有小步骤
                return True

        else:  # 没有的话直接运行
            return False

    def analysis_tasks(self, tasks, index=0, num=1):
        self.cur_num = 0
        self.temp = 1
        self.cur_index = "1"
        self.analysis_tasks_(tasks)
        self.judge_num = 0

    def run_(self, tasks, index=0, num=1, goal_index: str = "1", stop_index=None, is_continue=True):
        # num代表tasks处于字典嵌套的第几层
        goal_index_list = None
        goal_index_len = None
        if self.is_first:  # 为指定步骤的一些处理，为了第一次定位
            if self.is_judge:
                if self.temp_judge_num < self.judge_num:
                    cur_judge_key = judge_register(self.temp_judge_num, self.judge, self.cur_judge_key)
                    if cur_judge_key in self.index_list_dic:
                        self.cur_judge_key = cur_judge_key
                        # cur_index_list = self.index_list_dic[cur_judge_key]
                    cur_index_list = self.index_list_dic[self.cur_judge_key]
                    self.temp_judge_num += 1
                else:
                    if self.judge_num == 0:
                        if self.index_list_dic:
                            cur_index_list = self.index_list_dic['0']
                        else:
                            cur_index_list = self.index_list
                    else:
                        cur_index_list = self.index_list_dic[self.cur_judge_key]
            else:
                cur_index_list = self.index_list
            if goal_index not in cur_index_list:
                for i in range(3):
                    goal_index = reduce_index(goal_index)
                    if goal_index in cur_index_list:
                        index_ = cur_index_list.index(goal_index)
                        if index_ < len(cur_index_list) - 1:
                            goal_index = cur_index_list[index_ + 1]
                            self.cur_index = add_index(self.cur_index)
                            # num -= 1
                            # self.temp -= 1
                            break
                        else:
                            raise ValueError(f"index 没有找到, 已经跑完了")
            goal_index_list = analyse_index(goal_index)  # 分析目标索引
            goal_index_len = len(goal_index_list)
            if goal_index_len >= num:
                index = goal_index_list[num - 1] - 1  # 获取相关嵌套层数的索引
            else:
                index = 0

        if isinstance(tasks, dict):  # 是否有小步骤
            while True:
                if index < len(tasks):
                    for i in range(len(tasks) - index):

                        if num < self.temp:  # 检查是否返回了上一层的嵌套
                            self.temp = num
                            self.cur_index = index_by_reduce_num(self.cur_index, num)
                            if not is_continue:
                                return None
                        elif num > self.temp:  # 检查是否进入了下一层的嵌套
                            if not self.is_first or num > goal_index_len:
                                self.cur_index = index_by_add_num(self.cur_index)
                                self.temp = num
                            else:
                                self.temp = num

                        self.cur_num = num
                        goal = list(tasks.keys())[index]
                        task = tasks[goal]
                        self.color_print(f"{self.cur_index}  步骤 {goal}, 嵌套当前层数 {num}", is_bright=True)
                        if stop_index is not None:
                            if self.cur_index == stop_index:
                                return None

                        if self.is_first:
                            if goal_index_len > num:  # 检查是否到了目标索引的嵌套层数
                                self.cur_index += "." + str(goal_index_list[num])
                                result = self.run_(task, num=num + 1, goal_index=goal_index, is_continue=is_continue,
                                                   stop_index=stop_index)
                            else:
                                result = self.run_(task, index=0, num=num + 1, goal_index=goal_index,
                                                   is_continue=is_continue, stop_index=stop_index)
                        else:
                            result = self.run_(task, index=0, num=num + 1, is_continue=is_continue,
                                               stop_index=stop_index)

                        if result:
                            # 嵌套完成了，
                            index += 1
                            self.cur_index = add_index(self.cur_index)

                        elif result is False:
                            small_process_results = task.run()
                            if isinstance(small_process_results, tuple):
                                small_process_result = small_process_results[0]
                                self.judge_(small_process_results[1])

                            else:
                                small_process_result = small_process_results

                            if small_process_result:
                                index += 1
                                # if index != len(tasks):
                                self.cur_index = add_index(self.cur_index)
                                if not is_continue:
                                    if index == len(tasks):
                                        # 代表完成这一步的所有小步骤
                                        return True
                                    else:
                                        return None  # 打断
                            else:
                                continue
                        else:
                            return None

                        if stop_index is not None:
                            if self.cur_index == stop_index:
                                return None
                        if index == len(tasks):
                            # 代表完成这一步的所有小步骤
                            return True
                else:
                    return None
                    # time.sleep(3)
                    #
        elif isinstance(tasks, list):
            if self.judge is not None:
                task = tasks[self.judge]
                if not is_continue:
                    result = self.run_(task, index=goal_index_list[-1], num=num, goal_index=goal_index,
                                       is_continue=is_continue, stop_index=stop_index)
                else:
                    result = self.run_(task, index=0, num=num, goal_index=goal_index, is_continue=is_continue,
                                       stop_index=stop_index)
                if result:
                    # 嵌套完成了，
                    index += 1
                    self.cur_index = add_index(self.cur_index)
                elif result is False:
                    small_process_results = task.run()
                    if isinstance(small_process_results, tuple):
                        small_process_result = small_process_results[0]
                        self.judge = self.judge_(small_process_results[1])
                    else:
                        small_process_result = small_process_results

                    if small_process_result:
                        index += 1
                        # if index != len(tasks):
                        self.cur_index = add_index(self.cur_index)
                        if not is_continue:
                            if index == len(tasks):

                                # 代表完成这一步的所有小步骤
                                return True
                            else:
                                return None  # 打断

                else:
                    return None

                if stop_index is not None:
                    if self.cur_index == stop_index:
                        return None
                if index == 1:
                    # 代表完成这一步的所有小步骤
                    return True
            else:
                raise ValueError("没有选择")

        else:  # 没有的话直接运行
            self.is_first = False
            return False

    def run(self, goal_index: str = "1", stop_index=None):
        if self.tasks is None:
            raise ValueError("没有任务")
        if stop_index is not None:
            if stop_index not in self.index_list:
                raise ValueError(f"stop_index 不在任务列表中")
            else:
                stop_index = self.index_list.index(stop_index)
                if stop_index == self.index_len:
                    stop_index = self.index_list[stop_index]
                else:
                    stop_index = self.index_list[stop_index + 1]

        self.cur_num = 0
        self.temp = 1
        temp_goal = goal_index
        self.cur_index = goal_index[0]
        self.is_first = True
        # try:
        self.run_(self.tasks, goal_index=goal_index, stop_index=stop_index)
        # except:
        #     self.cur_index = temp_goal

    def reset(self):
        self.count = 0

    def run_once(self, goal_index: str="1"):
        is_continue = False
        if self.tasks is None:
            raise ValueError("没有任务")

        self.cur_num = 0
        self.temp = 1
        self.is_first = True
        if self.count == 0:
            temp_index = goal_index
            index_list = temp_index.split(".")
            self.cur_index = index_list[0]
            self.count += 1
        else:
            temp_index = self.cur_index
            index_list = self.cur_index.split(".")
            self.cur_index = index_list[0]
        # try:
        self.run_(self.tasks, goal_index=temp_index, is_continue=is_continue)

    def register(self, result):
        self.return_result = result

    def get_result(self):
        temp = self.return_result
        # self.return_result = None
        return temp


class Procedure_Action:
    procedure = None

    def __init__(self, procedure):
        Procedure_Action.procedure = procedure

    class Action:
        def __init__(self, action, *args, **kwargs):
            self.action = action
            self.args = args
            self.kwargs = kwargs
            self.original_print = builtins.print
            self.result = None
            self.is_judge = False
            if "sleep_time" in kwargs:
                self.sleep_time = kwargs.pop("sleep_time")
            else:
                self.sleep_time = None

            if "is_judge" in kwargs:
                kwargs.pop("is_judge")
                self.is_judge = True

        def run(self, *args, **kwargs):
            if "is_get_result" in self.kwargs:  # 是否要得到前面步骤的返回值
                if self.kwargs["is_get_result"]:
                    self.result = Procedure_Action.procedure.get_result()   # 得到前面步骤的返回值
                    self.kwargs.pop("is_get_result")
            # with replace_print():
            if self.result is not None:
                result = self.action(*self.result, **self.kwargs)
            else:
                if args and kwargs:
                    result = self.action(*args, **kwargs)
                elif args:
                    result = self.action(*args, **self.kwargs)
                elif kwargs:
                    result = self.action(*self.args, **kwargs)
                else:
                    result = self.action(*self.args, **self.kwargs)

            if self.sleep_time is not None:
                time.sleep(self.sleep_time)

            if isinstance(result, tuple):
                if self.is_judge:   # 是否需要判断
                    if Procedure_Action.procedure.temp_judge_num >= Procedure_Action.procedure.judge_num:
                        Procedure_Action.procedure.judge_num += 1
                    Procedure_Action.procedure.register(result[2:])
                    return result[0], result[1]     # 第一个值为程序是否继续，第二个值为判断值
                else:
                    Procedure_Action.procedure.register(result[1:])
                    return result[0]
            else:
                return result


class ThreadPool:
    def __init__(self, num=6):
        self.pool = ThreadPoolExecutor(max_workers=num)
        self.tasks = {}
        self.create_group(["catch", "aim", "put_down", "socket", "detect"])
        self.local = thread_local

    def logit(self, func, info, group):
        @wraps(func)
        def wrapper(*args, **kwargs):
            current_thread = threading.current_thread()
            self.local.info = info
            self.local.group = group
            return func(*args, **kwargs)
        return wrapper

    def create_group(self, groups):
        if isinstance(groups, str):
            self.tasks[groups] = {}
        elif isinstance(groups, list):
            for group in groups:
                self.tasks[group] = {}
        else:
            raise ValueError("groups must be a string or a list")

    def add_task(self, func, *args, **kwargs):
        if "info" in kwargs:
            info = kwargs.pop("info")
        else:
            info = None
        if "group" in kwargs:
            group = kwargs.pop("group")
        else:
            group = None

        func = self.logit(func, info, group)
        future = self.pool.submit(func, *args, **kwargs)

        if info is not None and group is not None:
            self.tasks[group][info] = future

        elif info is not None:
            self.tasks[info] = future

    def remove_task(self, info, group):
        if group in self.tasks:
            if info in self.tasks[group]:
                del self.tasks[group][info]
            else:
                print(f"Task with info {info} not found in group {group}")

        else:
            print(f"Group with name {group} not found")

    def wait(self, info, group):
        # print(self.tasks)
        if group in self.tasks:
            if info in self.tasks[group]:
                result = self.tasks[group][info].result()
                self.remove_task(info, group)
                return result
        print(f"Task with info {info} not found in group {group}")
        return None

    def wait_completion(self):
        self.pool.shutdown(wait=True)


# class Action:
#     def __init__(self, action, *args, **kwargs):
#         self.action = action
#         self.args = args
#         self.kwargs = kwargs
#         if "sleep_time" in kwargs:
#             self.sleep_time = kwargs.pop("sleep_time")
#         else:
#             self.sleep_time = None
#
#     def run(self, *args, **kwargs):
#         # with self.replace_print():
#         if args and kwargs:
#             result = self.action(*args, **kwargs)
#         elif args:
#             result = self.action(*args, **self.kwargs)
#         elif kwargs:
#             result = self.action(*self.args, **kwargs)
#         else:
#             result = self.action(*self.args, **self.kwargs)
#
#         if self.sleep_time is not None:
#             time.sleep(self.sleep_time)
#         return result
#
#     @contextlib.contextmanager
#     def replace_print(self):
#         original_print = builtins.print
#         self.procedure.original_print = original_print
#         try:
#             builtins.print = self.procedure.color_print
#             yield
#         finally:
#             builtins.print = original_print
