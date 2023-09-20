// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from explorer_interfaces:action/Wander.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "explorer_interfaces/action/detail/wander__struct.h"
#include "explorer_interfaces/action/detail/wander__type_support.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace explorer_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Wander_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Wander_Goal_type_support_ids_t;

static const _Wander_Goal_type_support_ids_t _Wander_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Wander_Goal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Wander_Goal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Wander_Goal_type_support_symbol_names_t _Wander_Goal_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, explorer_interfaces, action, Wander_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, explorer_interfaces, action, Wander_Goal)),
  }
};

typedef struct _Wander_Goal_type_support_data_t
{
  void * data[2];
} _Wander_Goal_type_support_data_t;

static _Wander_Goal_type_support_data_t _Wander_Goal_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Wander_Goal_message_typesupport_map = {
  2,
  "explorer_interfaces",
  &_Wander_Goal_message_typesupport_ids.typesupport_identifier[0],
  &_Wander_Goal_message_typesupport_symbol_names.symbol_name[0],
  &_Wander_Goal_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Wander_Goal_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Wander_Goal_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace explorer_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, explorer_interfaces, action, Wander_Goal)() {
  return &::explorer_interfaces::action::rosidl_typesupport_c::Wander_Goal_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "explorer_interfaces/action/detail/wander__struct.h"
// already included above
// #include "explorer_interfaces/action/detail/wander__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace explorer_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Wander_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Wander_Result_type_support_ids_t;

static const _Wander_Result_type_support_ids_t _Wander_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Wander_Result_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Wander_Result_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Wander_Result_type_support_symbol_names_t _Wander_Result_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, explorer_interfaces, action, Wander_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, explorer_interfaces, action, Wander_Result)),
  }
};

typedef struct _Wander_Result_type_support_data_t
{
  void * data[2];
} _Wander_Result_type_support_data_t;

static _Wander_Result_type_support_data_t _Wander_Result_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Wander_Result_message_typesupport_map = {
  2,
  "explorer_interfaces",
  &_Wander_Result_message_typesupport_ids.typesupport_identifier[0],
  &_Wander_Result_message_typesupport_symbol_names.symbol_name[0],
  &_Wander_Result_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Wander_Result_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Wander_Result_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace explorer_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, explorer_interfaces, action, Wander_Result)() {
  return &::explorer_interfaces::action::rosidl_typesupport_c::Wander_Result_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "explorer_interfaces/action/detail/wander__struct.h"
// already included above
// #include "explorer_interfaces/action/detail/wander__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace explorer_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Wander_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Wander_Feedback_type_support_ids_t;

static const _Wander_Feedback_type_support_ids_t _Wander_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Wander_Feedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Wander_Feedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Wander_Feedback_type_support_symbol_names_t _Wander_Feedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, explorer_interfaces, action, Wander_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, explorer_interfaces, action, Wander_Feedback)),
  }
};

typedef struct _Wander_Feedback_type_support_data_t
{
  void * data[2];
} _Wander_Feedback_type_support_data_t;

static _Wander_Feedback_type_support_data_t _Wander_Feedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Wander_Feedback_message_typesupport_map = {
  2,
  "explorer_interfaces",
  &_Wander_Feedback_message_typesupport_ids.typesupport_identifier[0],
  &_Wander_Feedback_message_typesupport_symbol_names.symbol_name[0],
  &_Wander_Feedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Wander_Feedback_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Wander_Feedback_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace explorer_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, explorer_interfaces, action, Wander_Feedback)() {
  return &::explorer_interfaces::action::rosidl_typesupport_c::Wander_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "explorer_interfaces/action/detail/wander__struct.h"
// already included above
// #include "explorer_interfaces/action/detail/wander__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace explorer_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Wander_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Wander_SendGoal_Request_type_support_ids_t;

static const _Wander_SendGoal_Request_type_support_ids_t _Wander_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Wander_SendGoal_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Wander_SendGoal_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Wander_SendGoal_Request_type_support_symbol_names_t _Wander_SendGoal_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, explorer_interfaces, action, Wander_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, explorer_interfaces, action, Wander_SendGoal_Request)),
  }
};

typedef struct _Wander_SendGoal_Request_type_support_data_t
{
  void * data[2];
} _Wander_SendGoal_Request_type_support_data_t;

static _Wander_SendGoal_Request_type_support_data_t _Wander_SendGoal_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Wander_SendGoal_Request_message_typesupport_map = {
  2,
  "explorer_interfaces",
  &_Wander_SendGoal_Request_message_typesupport_ids.typesupport_identifier[0],
  &_Wander_SendGoal_Request_message_typesupport_symbol_names.symbol_name[0],
  &_Wander_SendGoal_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Wander_SendGoal_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Wander_SendGoal_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace explorer_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, explorer_interfaces, action, Wander_SendGoal_Request)() {
  return &::explorer_interfaces::action::rosidl_typesupport_c::Wander_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "explorer_interfaces/action/detail/wander__struct.h"
// already included above
// #include "explorer_interfaces/action/detail/wander__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace explorer_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Wander_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Wander_SendGoal_Response_type_support_ids_t;

static const _Wander_SendGoal_Response_type_support_ids_t _Wander_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Wander_SendGoal_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Wander_SendGoal_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Wander_SendGoal_Response_type_support_symbol_names_t _Wander_SendGoal_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, explorer_interfaces, action, Wander_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, explorer_interfaces, action, Wander_SendGoal_Response)),
  }
};

typedef struct _Wander_SendGoal_Response_type_support_data_t
{
  void * data[2];
} _Wander_SendGoal_Response_type_support_data_t;

static _Wander_SendGoal_Response_type_support_data_t _Wander_SendGoal_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Wander_SendGoal_Response_message_typesupport_map = {
  2,
  "explorer_interfaces",
  &_Wander_SendGoal_Response_message_typesupport_ids.typesupport_identifier[0],
  &_Wander_SendGoal_Response_message_typesupport_symbol_names.symbol_name[0],
  &_Wander_SendGoal_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Wander_SendGoal_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Wander_SendGoal_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace explorer_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, explorer_interfaces, action, Wander_SendGoal_Response)() {
  return &::explorer_interfaces::action::rosidl_typesupport_c::Wander_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "explorer_interfaces/action/detail/wander__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace explorer_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Wander_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Wander_SendGoal_type_support_ids_t;

static const _Wander_SendGoal_type_support_ids_t _Wander_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Wander_SendGoal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Wander_SendGoal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Wander_SendGoal_type_support_symbol_names_t _Wander_SendGoal_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, explorer_interfaces, action, Wander_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, explorer_interfaces, action, Wander_SendGoal)),
  }
};

typedef struct _Wander_SendGoal_type_support_data_t
{
  void * data[2];
} _Wander_SendGoal_type_support_data_t;

static _Wander_SendGoal_type_support_data_t _Wander_SendGoal_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Wander_SendGoal_service_typesupport_map = {
  2,
  "explorer_interfaces",
  &_Wander_SendGoal_service_typesupport_ids.typesupport_identifier[0],
  &_Wander_SendGoal_service_typesupport_symbol_names.symbol_name[0],
  &_Wander_SendGoal_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t Wander_SendGoal_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Wander_SendGoal_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace explorer_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, explorer_interfaces, action, Wander_SendGoal)() {
  return &::explorer_interfaces::action::rosidl_typesupport_c::Wander_SendGoal_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "explorer_interfaces/action/detail/wander__struct.h"
// already included above
// #include "explorer_interfaces/action/detail/wander__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace explorer_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Wander_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Wander_GetResult_Request_type_support_ids_t;

static const _Wander_GetResult_Request_type_support_ids_t _Wander_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Wander_GetResult_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Wander_GetResult_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Wander_GetResult_Request_type_support_symbol_names_t _Wander_GetResult_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, explorer_interfaces, action, Wander_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, explorer_interfaces, action, Wander_GetResult_Request)),
  }
};

typedef struct _Wander_GetResult_Request_type_support_data_t
{
  void * data[2];
} _Wander_GetResult_Request_type_support_data_t;

static _Wander_GetResult_Request_type_support_data_t _Wander_GetResult_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Wander_GetResult_Request_message_typesupport_map = {
  2,
  "explorer_interfaces",
  &_Wander_GetResult_Request_message_typesupport_ids.typesupport_identifier[0],
  &_Wander_GetResult_Request_message_typesupport_symbol_names.symbol_name[0],
  &_Wander_GetResult_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Wander_GetResult_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Wander_GetResult_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace explorer_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, explorer_interfaces, action, Wander_GetResult_Request)() {
  return &::explorer_interfaces::action::rosidl_typesupport_c::Wander_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "explorer_interfaces/action/detail/wander__struct.h"
// already included above
// #include "explorer_interfaces/action/detail/wander__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace explorer_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Wander_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Wander_GetResult_Response_type_support_ids_t;

static const _Wander_GetResult_Response_type_support_ids_t _Wander_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Wander_GetResult_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Wander_GetResult_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Wander_GetResult_Response_type_support_symbol_names_t _Wander_GetResult_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, explorer_interfaces, action, Wander_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, explorer_interfaces, action, Wander_GetResult_Response)),
  }
};

typedef struct _Wander_GetResult_Response_type_support_data_t
{
  void * data[2];
} _Wander_GetResult_Response_type_support_data_t;

static _Wander_GetResult_Response_type_support_data_t _Wander_GetResult_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Wander_GetResult_Response_message_typesupport_map = {
  2,
  "explorer_interfaces",
  &_Wander_GetResult_Response_message_typesupport_ids.typesupport_identifier[0],
  &_Wander_GetResult_Response_message_typesupport_symbol_names.symbol_name[0],
  &_Wander_GetResult_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Wander_GetResult_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Wander_GetResult_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace explorer_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, explorer_interfaces, action, Wander_GetResult_Response)() {
  return &::explorer_interfaces::action::rosidl_typesupport_c::Wander_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "explorer_interfaces/action/detail/wander__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace explorer_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Wander_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Wander_GetResult_type_support_ids_t;

static const _Wander_GetResult_type_support_ids_t _Wander_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Wander_GetResult_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Wander_GetResult_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Wander_GetResult_type_support_symbol_names_t _Wander_GetResult_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, explorer_interfaces, action, Wander_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, explorer_interfaces, action, Wander_GetResult)),
  }
};

typedef struct _Wander_GetResult_type_support_data_t
{
  void * data[2];
} _Wander_GetResult_type_support_data_t;

static _Wander_GetResult_type_support_data_t _Wander_GetResult_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Wander_GetResult_service_typesupport_map = {
  2,
  "explorer_interfaces",
  &_Wander_GetResult_service_typesupport_ids.typesupport_identifier[0],
  &_Wander_GetResult_service_typesupport_symbol_names.symbol_name[0],
  &_Wander_GetResult_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t Wander_GetResult_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Wander_GetResult_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace explorer_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, explorer_interfaces, action, Wander_GetResult)() {
  return &::explorer_interfaces::action::rosidl_typesupport_c::Wander_GetResult_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "explorer_interfaces/action/detail/wander__struct.h"
// already included above
// #include "explorer_interfaces/action/detail/wander__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace explorer_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Wander_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Wander_FeedbackMessage_type_support_ids_t;

static const _Wander_FeedbackMessage_type_support_ids_t _Wander_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Wander_FeedbackMessage_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Wander_FeedbackMessage_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Wander_FeedbackMessage_type_support_symbol_names_t _Wander_FeedbackMessage_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, explorer_interfaces, action, Wander_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, explorer_interfaces, action, Wander_FeedbackMessage)),
  }
};

typedef struct _Wander_FeedbackMessage_type_support_data_t
{
  void * data[2];
} _Wander_FeedbackMessage_type_support_data_t;

static _Wander_FeedbackMessage_type_support_data_t _Wander_FeedbackMessage_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Wander_FeedbackMessage_message_typesupport_map = {
  2,
  "explorer_interfaces",
  &_Wander_FeedbackMessage_message_typesupport_ids.typesupport_identifier[0],
  &_Wander_FeedbackMessage_message_typesupport_symbol_names.symbol_name[0],
  &_Wander_FeedbackMessage_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Wander_FeedbackMessage_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Wander_FeedbackMessage_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace explorer_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, explorer_interfaces, action, Wander_FeedbackMessage)() {
  return &::explorer_interfaces::action::rosidl_typesupport_c::Wander_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "action_msgs/msg/goal_status_array.h"
#include "action_msgs/srv/cancel_goal.h"
#include "explorer_interfaces/action/wander.h"
// already included above
// #include "explorer_interfaces/action/detail/wander__type_support.h"

static rosidl_action_type_support_t _explorer_interfaces__action__Wander__typesupport_c;

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_action_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME(
  rosidl_typesupport_c, explorer_interfaces, action, Wander)()
{
  // Thread-safe by always writing the same values to the static struct
  _explorer_interfaces__action__Wander__typesupport_c.goal_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, explorer_interfaces, action, Wander_SendGoal)();
  _explorer_interfaces__action__Wander__typesupport_c.result_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, explorer_interfaces, action, Wander_GetResult)();
  _explorer_interfaces__action__Wander__typesupport_c.cancel_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, action_msgs, srv, CancelGoal)();
  _explorer_interfaces__action__Wander__typesupport_c.feedback_message_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c, explorer_interfaces, action, Wander_FeedbackMessage)();
  _explorer_interfaces__action__Wander__typesupport_c.status_message_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c, action_msgs, msg, GoalStatusArray)();

  return &_explorer_interfaces__action__Wander__typesupport_c;
}

#ifdef __cplusplus
}
#endif
