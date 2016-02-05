/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef _GAZEBO_UTIL_INTROSPECTION_CLIENT_HH_
#define _GAZEBO_UTIL_INTROSPECTION_CLIENT_HH_

#include <chrono>
#include <memory>
#include <set>
#include <string>
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace util
  {
    // Forward declare private data classes.
    class IntrospectionClientPrivate;

    /// addtogroup gazebo_util
    /// \{

    /// \class IntrospectionClient IntrospectionClient.hh util/util.hh
    /// \brief
    class GZ_UTIL_VISIBLE IntrospectionClient
    {
      /// \brief Constructor.
      public: IntrospectionClient();

      /// \brief Destructor.
      public: virtual ~IntrospectionClient();

      /// \brief Wait for introspection managers to appear on the network.
      /// This function is useful for clients that want to pause until
      /// one or more IntrospectionManagers becomes available (such as
      /// when gzserver is run).
      /// \param[in] _timeOut Maximum duration to wait. A value of
      /// std::chrono::duration::zero indicates that the function should
      /// wait indefinitely. An indefinite wait is the default behavior.
      /// \return List of available managers.
      /// \sa Managers()
      public: std::set<std::string> WaitForManagers(
                  const std::chrono::milliseconds _timeOut =
                  std::chrono::milliseconds::zero()) const;

      /// \brief Get the list of introspection managers currently available.
      /// \return Set of unique manager IDs.
      public: std::set<std::string> Managers() const;

      /// \brief Create a new filter for observing item updates. This function
      /// will create a new topic for sending periodic updates of the items
      /// specified in the filter.
      /// \param[in] _managerID ID of the manager to request the operation.
      /// \param[in] _newItems Non-empty set of items to observe.
      /// \param[out] _filterId Unique ID of the filter. You'll need this ID
      /// for future filter updates or for removing it.
      /// \param[out] _newTopic After the filter creation, a client should
      /// subscribe to this topic for receiving updates.
      /// \return True if the filter was successfully created or false otherwise
      public: bool NewFilter(const std::string &_managerId,
                             const std::set<std::string> &_newItems,
                             std::string &_filterId,
                             std::string &_newTopic) const;

      /// \brief Update an existing filter with a different set of items.
      /// \param[in] _managerID ID of the manager to request the operation.
      /// \param[in] _filterId ID of the filter to update.
      /// \param[in] _NewItems Non-empty set of items to be observed.
      /// \return True if the filter was successfuly updated or false otherwise.
      public: bool UpdateFilter(const std::string &_managerId,
                                const std::string &_filterId,
                                const std::set<std::string> &_newItems) const;

      /// \brief Remove all existing filters.
      /// \return True if the filters wer successfully removed
      /// or false otherwise
      public: bool RemoveAllFilters() const;

      /// \brief Remove an existing filter.
      /// \param[in] _managerID ID of the manager to request the operation.
      /// \param[in] _filterId ID of the filter to remove.
      /// \return True if the filter was successfully removed or false otherwise
      public: bool RemoveFilter(const std::string &_managerId,
                                const std::string &_filterId) const;

      /// \brief Get a copy of the items already registered.
      /// \param[in] _managerID ID of the manager to request the operation.
      /// \param[out] _items The list of items.
      /// \return True if the operation succeed or false otherwise
      /// (wrong managerID, transport problem).
      public: bool Items(const std::string &_managerId,
                         std::set<std::string> &_items) const;

      /// \bried Check if the _item is registered on a manager with
      /// _managerId.
      /// \param[in] _managerId Id of the manager to query.
      /// \param[in] _item Item name for the query.
      /// \return True if the introspection manager with ID==_managerId has
      /// an item registered with name == _item.
      public: bool IsRegistered(const std::string &_managerId,
                                const std::string &_item) const;

      /// \bried Check if the _items are registered on a manager with
      /// _managerId.
      /// \param[in] _managerId Id of the manager to query.
      /// \param[in] _items Set of item names for the query.
      /// \return True if the introspection manager with ID==_managerId has
      /// all the items registered.
      public: bool IsRegistered(const std::string &_managerId,
                                const std::set<std::string> &_items) const;

      /// \internal
      /// \brief Private data pointer.
      private: std::unique_ptr<IntrospectionClientPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
