
import 'dart:async';

import 'package:flutter/services.dart';

class Opencv4 {
  static const MethodChannel _channel = MethodChannel('opencv4');

  static Future<String?> get platformVersion async {
    final String? version = await _channel.invokeMethod('getPlatformVersion');
    return version;
  }

  static Future<String?> doSomething({
    required String appId,
    String? todo,
  }) async {
    final String? version = await _channel.invokeMethod(
      'doSomething',
      {
        'appId': appId,
        'todo': todo,
      }
    );
    return version;
  }
}
