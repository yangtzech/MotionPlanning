"""
控制器日志记录器，使用 loguru 库实现，支持控制台输出和文件存储
"""

import os
import sys
from datetime import datetime
from enum import Enum

from loguru import logger


class LogLevel(Enum):
    """日志级别枚举"""

    DEBUG = "DEBUG"
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"
    CRITICAL = "CRITICAL"


class ControllerLogger:
    """控制器专用日志记录器"""

    def __init__(self, name: str = "Controller", log_dir: str = "logs"):
        # 移除默认处理器
        logger.remove()

        # 创建日志目录
        if not os.path.exists(log_dir):
            os.makedirs(log_dir, exist_ok=True)

        # 生成带时间戳的日志文件名
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_file = os.path.join(log_dir, f"{name}_{timestamp}.log")

        # 添加文件处理器
        logger.add(
            log_file,
            format="{time:YYYY-MM-DD HH:mm:ss} - {level} - " + name + " - {message}",
            level="DEBUG",
            rotation="10 MB",  # 当日志文件达到10MB时自动轮转
            retention="7 days",  # 保留最近7天的日志文件
        )

        # 添加标准输出处理器，格式化输出
        logger.add(
            sys.stdout,
            format="{time:YYYY-MM-DD HH:mm:ss} - {level} - " + name + " - {message}",
            level="INFO",
            colorize=True,  # 支持彩色输出
        )

        self._logger = logger
        self.log_file = log_file

    def debug(self, message: str):
        """调试信息"""
        self._logger.debug(message)

    def info(self, message: str):
        """一般信息"""
        self._logger.info(message)

    def warning(self, message: str):
        """警告信息"""
        self._logger.warning(message)

    def error(self, message: str):
        """错误信息"""
        self._logger.error(message)

    def critical(self, message: str):
        """严重错误信息"""
        self._logger.critical(message)

    def set_level(self, level: str):
        """设置日志级别"""
        self._logger.level = level


# 全局控制器日志记录器实例
controller_logger = ControllerLogger()


def setup_logger(
    name: str = "Controller",
    log_dir: str = "logs",
    console_level: str = "INFO",
    file_level: str = "DEBUG",
):
    """设置日志记录器，支持控制台和文件双输出"""
    # 移除所有现有处理器
    logger.remove()

    # 创建日志目录
    if not os.path.exists(log_dir):
        os.makedirs(log_dir, exist_ok=True)

    # 生成带时间戳的日志文件名
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file = os.path.join(log_dir, f"{name}_{timestamp}.log")

    # 添加文件处理器
    logger.add(
        log_file,
        format="{time:YYYY-MM-DD HH:mm:ss} - {level} - " + name + " - {message}",
        level=file_level,
        rotation="10 MB",  # 当日志文件达到10MB时自动轮转
        retention="7 days",  # 保留最近7天的日志文件
        compression="zip",  # 压缩旧日志文件
    )

    # 添加标准输出处理器
    logger.add(
        sys.stdout,
        format="{time:YYYY-MM-DD HH:mm:ss} - {level} - " + name + " - {message}",
        level=console_level,
        colorize=True,  # 支持彩色输出
    )

    return log_file


def set_controller_log_level(level: str = "INFO"):
    """全局设置控制器日志级别"""
    setup_logger("Controller", console_level=level)


def log_controller_debug(message: str, enabled: bool = True):
    """条件性控制器调试日志记录"""
    if enabled:
        logger.debug(message)


def log_controller_info(message: str, enabled: bool = True):
    """条件性控制器信息日志记录"""
    if enabled:
        logger.info(message)


def log_controller_warning(message: str):
    """控制器警告日志记录"""
    logger.warning(message)


def log_controller_error(message: str):
    """控制器错误日志记录"""
    logger.error(message)


def log_controller_exception(message: str, exc_info: bool = True):
    """控制器异常日志记录"""
    if exc_info:
        logger.exception(message)
    else:
        logger.error(message)
