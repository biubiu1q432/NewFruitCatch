"""
    对 run_ 里索引的一些处理
"""


def analyse_index(index):
    """
    分析索引
    :param index:
    :return: 数字列表
    """
    digits = []
    for digit in index.split("."):
        digits.append(int(digit))
    return digits


def add_index(cur_index:str):
    """
    给索引加1
    :param cur_index:
    :return:
    """
    index_list = cur_index.split(".")
    number = index_list[-1]
    number = int(number) + 1
    index_list[-1] = str(number)
    return ".".join(index_list)


def index_by_reduce_num(index, num):
    """
    根据嵌套的层数来减少索引的层数
    :param index: 索引
    :param num: 嵌套的层数
    :return: 新的索引
    """
    index_list = index.split(".")
    while len(index_list) > num:
        index_list.pop()
    number = index_list[-1]
    number = int(number) + 1
    index_list[-1] = str(number)
    return ".".join(index_list)


def index_by_add_num(index):
    """
    根据索引的层数来增加索引的层数
    :param index: 索引
    :return: 新的索引
    """
    index_list = index.split(".")
    index_list.append("1")
    return ".".join(index_list)


def judge_register(judge_num, i, cur_judge_key):
    if judge_num == 0 or judge_num is None:
        return str(i)
    else:
        return cur_judge_key + "-" + str(i)


def reduce_index(cur_index: str):
    index_list = cur_index.split(".")
    number = index_list[-1]
    number = int(number) - 1
    index_list[-1] = str(number)
    return ".".join(index_list)