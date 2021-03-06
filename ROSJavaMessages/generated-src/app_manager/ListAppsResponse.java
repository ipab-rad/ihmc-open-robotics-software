package app_manager;

public interface ListAppsResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "app_manager/ListAppsResponse";
  static final java.lang.String _DEFINITION = "App[] running_apps\nApp[] available_apps";
  java.util.List<app_manager.App> getRunningApps();
  void setRunningApps(java.util.List<app_manager.App> value);
  java.util.List<app_manager.App> getAvailableApps();
  void setAvailableApps(java.util.List<app_manager.App> value);
}
