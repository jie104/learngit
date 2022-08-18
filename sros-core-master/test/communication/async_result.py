#!/usr/bin/env python
# -*- coding:utf-8 -*-
# file async_result.py
# author pengjiali
# date 19-7-1.
# copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
# describe

from enum import Enum
import asyncio


class AsyncResultState(Enum):
    '''结果状态枚举'''
    NONE = 0
    ACCEPT = 1
    REJECT = 2


class AsyncResult:
    '''
    异步等待结果,支持多线程中设置结果
    NOTE: asyncio.Event不是线程安全的，在A线程中等待结果 awit Event.wait()，在B线程中设置结果Event.set(),
            虽然Event.is_set() == True，但是A线程会一直等待不会结束。
    '''

    def __init__(self, loop):
        self._loop = loop
        self._event = asyncio.Event(loop=loop)
        self._result = None
        self._result_state = AsyncResultState.NONE

    def clear(self):
        self._event.clear()
        self._result = None
        self._result_state = AsyncResultState.NONE

    async def wait(self):
        await self._event.wait()
        if self._result_state == None:
            raise Exception('UNREACHABLE')
        elif self._result_state == AsyncResultState.ACCEPT:
            return self._result
        else:
            raise RuntimeError("aync request reject, error code: ", self._result)

    def reject(self, result):
        def fun(self):
            self._result_state = AsyncResultState.REJECT
            self._result = result
            self._event.set()

        self._loop.call_soon_threadsafe(fun, self)

    def accept(self, result):
        def fun(self):
            self._result_state = AsyncResultState.ACCEPT
            self._result = result
            self._event.set()

        self._loop.call_soon_threadsafe(fun, self)


if __name__ == '__main__':
    import threading
    import time

    loop = asyncio.new_event_loop()
    async_result = AsyncResult(loop)


    async def action(async_result):
        async_result.clear()
        # do something
        print('wait result', threading.current_thread())
        return await async_result.wait()


    def new_result(async_result):
        time.sleep(1)
        async_result.accept(100)
        print('new result 100', threading.current_thread())


    f = action(async_result)
    # loop.call_later(1, new_result, async_result) # result fun call in the same thread
    threading.Thread(target=new_result, args=(async_result,)).start()  # result call in different thread
    loop.run_until_complete(f)
    loop.close()
