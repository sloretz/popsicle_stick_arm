/*
Popsicle stick robot
Copyright (C) 2015  Shane Loretz

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef VARIABLE_ARRAY_H
#define VARIABLE_ARRAY_H

#include <stdlib.h>

template <typename T>
class VariableArray
{
    public:
        VariableArray()
        {
            size_ = 0;
            capacity_ = 0;
            array_ = 0;
        }
        ~VariableArray()
        {
            clear();
        }
        
        void reserve(unsigned int amount)
        {
            if (capacity_ == amount)
            {
                return;
            }
            else if (capacity_ == 0)
            {
                //initialize the array;
                array_ = (T*) malloc(sizeof(T)*amount);
            }
            else if (amount == 0)
            {
                //free the array
                free(array_);
            }
            else
            {
                //Shrink if less than current capacity
                //or grow to a larger capacity
                array_ = (T*)realloc(array_, sizeof(T)*amount);
            }
            capacity_ = amount;
            if (amount < size_)
            {
                //truncate if new size is smaller
                size_ = amount;
            }
        }
        
        void push_back(T &val)
        {
            if (capacity_ <= size_)
            {
                //grow 3 elements at a time
                reserve(capacity_+3);
            }
            array_[size_] = val;
            size_++;
        }
        
        T& operator[](int index)
        {
            return array_[index];
        }
        
        void clear()
        {
            reserve(0);
        }
        
        unsigned int size()
        {
            return size_;
        }

    protected:
        T *array_;
        unsigned int size_;
        unsigned int capacity_;
};

#endif
