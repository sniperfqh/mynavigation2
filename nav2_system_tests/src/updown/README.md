# Nav2 Updown Test

This is a 'top level' system test which tests the lifecycle bringup and shutdown of the system. 

## To run the test
```

## 中文翻译

# Nav2 Updown 测试

这是一个顶层系统测试，用于验证整套导航系统的 Lifecycle 启动和关闭。运行 test_updown_launch.py 后，终端应出现 TEST PASSED。需要循环执行 1000 次时，可运行 test_updown_reliablity 保存日志，再用 updownresults.py 汇总结果。
ros2 launch nav2_system_tests test_updown_launch.py
```

If the test passes, you should see this comment in the output:
```
[test_updown-13] [INFO] [test_updown]: ****************************************************  TEST PASSED!
```

To run the test in a loop 1000x, run the `test_updown_reliablity` script and log the output:
```
./test_updown_reliablity |& tee /tmp/updown.log
```
When the test is completed, pipe the log to the `updownresults.py` script to get a summary of the results:
```
./updownresults.py < /tmp/updown.log`
```
