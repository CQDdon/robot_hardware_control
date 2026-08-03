#include "hardware_ipc_common/shared_memory.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <utility>
#include <cstdio>

namespace hardware_ipc_common
{

SharedMemorySegment::SharedMemorySegment(std::string name, std::size_t size)
	: name_(std::move(name)), size_(size)
{
}

SharedMemorySegment::SharedMemorySegment(SharedMemorySegment &&other) noexcept
{
	*this = std::move(other);
}

SharedMemorySegment &SharedMemorySegment::operator=(SharedMemorySegment &&other) noexcept
{
	if (this == &other)
		return *this;

	close();

	name_ = std::move(other.name_);
	size_ = other.size_;
	fd_ = other.fd_;
	ptr_ = other.ptr_;

	other.size_ = 0;
	other.fd_ = -1;
	other.ptr_ = nullptr;

	return *this;
}

SharedMemorySegment::~SharedMemorySegment()
{
	close();
}

bool SharedMemorySegment::open(bool create_if_missing)
{
	close();

	int flags = O_RDWR;
	if (create_if_missing)
	{
		flags |= O_CREAT;
	}

	fd_ = shm_open(name_.c_str(), flags, 0666);
	if (fd_ < 0)
	{
        std::fprintf(stderr, "ERROR: could not create FD, errno= %s\n",std::strerror(errno));
		return false;
	}

	if (ftruncate(fd_, static_cast<off_t>(size_)) != 0)
	{
		::close(fd_);
		fd_ = -1;
        std::fprintf(stderr, "ERROR: could ftruncate FD, errno=%s\n", std::strerror(errno));
		return false;
	}

	void *addr = mmap(nullptr, size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
	if (addr == MAP_FAILED)
	{
		::close(fd_);
		fd_ = -1;
        std::fprintf(stderr, "ERROR: could not create ADDRESS, errno=%s\n", std::strerror(errno));
		return false;
	}

	ptr_ = addr;
	return true;
}

void SharedMemorySegment::close()
{
	if (ptr_)
	{
		munmap(ptr_, size_);
		ptr_ = nullptr;
	}

	if (fd_ >= 0)
	{
		::close(fd_);
		fd_ = -1;
	}
}

void SharedMemorySegment::reset()
{
	close();
	name_.clear();
	size_ = 0;
}

SharedMemoryInterface::~SharedMemoryInterface()
{
	close();
}

bool SharedMemoryInterface::open(bool clear_on_open)
{
	if (!cmd_if_segment_.open())
	{
		return false;
	}

	if (!state_segment_.open())
	{
		cmd_if_segment_.close();
		return false;
	}

	if (!state_debug_segment_.open())
	{
		cmd_if_segment_.close();
		state_segment_.close();
		return false;
	}

	if (!sensor_state_segment_.open())
	{
		cmd_if_segment_.close();
		state_segment_.close();
		state_debug_segment_.close();
		return false;
	}

	if (!control_segment_.open())
	{
		cmd_if_segment_.close();
		state_segment_.close();
		state_debug_segment_.close();
		sensor_state_segment_.close();
		return false;
	}

	if (clear_on_open)
	{
		initialize_snapshot_block(cmd_if_segment_.as<ShmSnapshotBlock<HwiCommandIfBlock>>());
		initialize_snapshot_block(state_segment_.as<ShmSnapshotBlock<HwiStateBlock>>());
		initialize_snapshot_block(state_debug_segment_.as<ShmSnapshotBlock<HwiStateDebugBlock>>());
		initialize_snapshot_block(sensor_state_segment_.as<ShmSnapshotBlock<HwiSensorStateBlock>>());
		initialize_snapshot_block(control_segment_.as<ShmSnapshotBlock<HshControlBlock>>());
	}
	else if (!snapshot_block_valid(cmd_if_segment_.as<ShmSnapshotBlock<HwiCommandIfBlock>>()) ||
	         !snapshot_block_valid(state_segment_.as<ShmSnapshotBlock<HwiStateBlock>>()) ||
	         !snapshot_block_valid(state_debug_segment_.as<ShmSnapshotBlock<HwiStateDebugBlock>>()) ||
	         !snapshot_block_valid(sensor_state_segment_.as<ShmSnapshotBlock<HwiSensorStateBlock>>()) ||
	         !snapshot_block_valid(control_segment_.as<ShmSnapshotBlock<HshControlBlock>>()))
	{
		close();
		return false;
	}
	return true;
}

bool SharedMemoryInterface::write_state(const HwiStateBlock &state_data)
{
	if (!ready())
	{
		return false;
	}

	return write_snapshot_block(state_segment_.as<ShmSnapshotBlock<HwiStateBlock>>(), state_data);
}

bool SharedMemoryInterface::write_state_debug(const HwiStateDebugBlock &state_data)
{
	if (!ready())
	{
		return false;
	}

	return write_snapshot_block(state_debug_segment_.as<ShmSnapshotBlock<HwiStateDebugBlock>>(), state_data);
}

bool SharedMemoryInterface::write_sensor_state(const HwiSensorStateBlock &state_data)
{
	if (!ready())
	{
		return false;
	}

	return write_snapshot_block(sensor_state_segment_.as<ShmSnapshotBlock<HwiSensorStateBlock>>(), state_data);
}

bool SharedMemoryInterface::write_cmd_if(const HwiCommandIfBlock &command_if_data)
{
	if (!ready())
	{
		return false;
	}

	return write_snapshot_block(cmd_if_segment_.as<ShmSnapshotBlock<HwiCommandIfBlock>>(), command_if_data);
}

bool SharedMemoryInterface::write_control(const HshControlBlock &control_data)
{
	if (!ready())
	{
		return false;
	}

	return write_snapshot_block(control_segment_.as<ShmSnapshotBlock<HshControlBlock>>(), control_data);
}

void SharedMemoryInterface::close()
{
	cmd_if_segment_.close();
	state_segment_.close();
	state_debug_segment_.close();
	sensor_state_segment_.close();
	control_segment_.close();
}

bool SharedMemoryInterface::ready() const noexcept
{
	return cmd_if_segment_.is_open() && state_segment_.is_open() &&
	       state_debug_segment_.is_open() && sensor_state_segment_.is_open() &&
	       control_segment_.is_open();
}

bool SharedMemoryInterface::read_cmd_if(HwiCommandIfBlock &command_if) const
{
	return ready() && read_snapshot_block(cmd_if_segment_.as<const ShmSnapshotBlock<HwiCommandIfBlock>>(), command_if, &snapshot_stats_);
}

bool SharedMemoryInterface::read_state(HwiStateBlock &state_data) const
{
	return ready() && read_snapshot_block(state_segment_.as<const ShmSnapshotBlock<HwiStateBlock>>(), state_data, &snapshot_stats_);
}

bool SharedMemoryInterface::read_state_debug(HwiStateDebugBlock &state_data) const
{
	return ready() && read_snapshot_block(state_debug_segment_.as<const ShmSnapshotBlock<HwiStateDebugBlock>>(), state_data, &snapshot_stats_);
}

bool SharedMemoryInterface::read_sensor_state(HwiSensorStateBlock &state_data) const
{
	return ready() && read_snapshot_block(sensor_state_segment_.as<const ShmSnapshotBlock<HwiSensorStateBlock>>(), state_data, &snapshot_stats_);
}

bool SharedMemoryInterface::read_control(HshControlBlock &control_data) const
{
	return ready() && read_snapshot_block(control_segment_.as<const ShmSnapshotBlock<HshControlBlock>>(), control_data, &snapshot_stats_);
}

} // namespace hardware_ipc_common
