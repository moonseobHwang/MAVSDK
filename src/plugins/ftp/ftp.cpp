// WARNING: THIS FILE IS AUTOGENERATED! As such, it should not be edited.
// Edits need to be made to the proto files
// (see https://github.com/mavlink/MAVSDK-Proto/blob/master/protos/ftp/ftp.proto)

#include <iomanip>

#include "ftp_impl.h"
#include "plugins/ftp/ftp.h"

namespace mavsdk {

using ProgressData = Ftp::ProgressData;

Ftp::Ftp(System& system) : PluginBase(), _impl{new FtpImpl(system)} {}

Ftp::~Ftp() {}

void Ftp::reset_async(const result_callback_t callback)
{
    _impl->reset_async(callback);
}

void Ftp::download_async(
    std::string remote_file_path, std::string local_dir, download_callback_t callback)
{
    _impl->download_async(remote_file_path, local_dir, callback);
}

void Ftp::upload_async(
    std::string local_file_path, std::string remote_dir, upload_callback_t callback)
{
    _impl->upload_async(local_file_path, remote_dir, callback);
}

void Ftp::list_directory_async(std::string remote_dir, const list_directory_callback_t callback)
{
    _impl->list_directory_async(remote_dir, callback);
}

void Ftp::create_directory_async(std::string remote_dir, const result_callback_t callback)
{
    _impl->create_directory_async(remote_dir, callback);
}

void Ftp::remove_directory_async(std::string remote_dir, const result_callback_t callback)
{
    _impl->remove_directory_async(remote_dir, callback);
}

void Ftp::remove_file_async(std::string remote_file_path, const result_callback_t callback)
{
    _impl->remove_file_async(remote_file_path, callback);
}

void Ftp::rename_async(
    std::string remote_from_path, std::string remote_to_path, const result_callback_t callback)
{
    _impl->rename_async(remote_from_path, remote_to_path, callback);
}

void Ftp::are_files_identical_async(
    std::string local_file_path,
    std::string remote_file_path,
    const are_files_identical_callback_t callback)
{
    _impl->are_files_identical_async(local_file_path, remote_file_path, callback);
}

Ftp::Result Ftp::set_root_directory(std::string root_dir) const
{
    return _impl->set_root_directory(root_dir);
}

Ftp::Result Ftp::set_target_component_id(uint32_t component_id) const
{
    return _impl->set_target_component_id(component_id);
}

uint32_t Ftp::get_our_component_id() const
{
    return _impl->get_our_component_id();
}

bool operator==(const Ftp::ProgressData& lhs, const Ftp::ProgressData& rhs)
{
    return (rhs.bytes_transferred == lhs.bytes_transferred) && (rhs.total_bytes == lhs.total_bytes);
}

std::ostream& operator<<(std::ostream& str, Ftp::ProgressData const& progress_data)
{
    str << std::setprecision(15);
    str << "progress_data:" << '\n' << "{\n";
    str << "    bytes_transferred: " << progress_data.bytes_transferred << '\n';
    str << "    total_bytes: " << progress_data.total_bytes << '\n';
    str << '}';
    return str;
}

const char* Ftp::result_str(Ftp::Result result)
{
    switch (result) {
        case Ftp::Result::Unknown:
            return "Unknown error";
        case Ftp::Result::Success:
            return "Success";
        case Ftp::Result::Next:
            return "Intermediate message showing progress";
        case Ftp::Result::Timeout:
            return "Timeout";
        case Ftp::Result::Busy:
            return "Operation is already in progress";
        case Ftp::Result::FileIoError:
            return "File IO operation error";
        case Ftp::Result::FileExists:
            return "File exists already";
        case Ftp::Result::FileDoesNotExist:
            return "File does not exist";
        case Ftp::Result::FileProtected:
            return "File is write protected";
        case Ftp::Result::InvalidParameter:
            return "Invalid parameter";
        case Ftp::Result::Unsupported:
            return "Unsupported command";
        case Ftp::Result::ProtocolError:
            return "General protocol error";
        default:
            return "Unknown";
    }
}

std::ostream& operator<<(std::ostream& str, Ftp::Result const& result)
{
    switch (result) {
        case Ftp::Result::Unknown:
            return str << "Result Unknown";
        case Ftp::Result::Success:
            return str << "Result Success";
        case Ftp::Result::Next:
            return str << "Result Next";
        case Ftp::Result::Timeout:
            return str << "Result Timeout";
        case Ftp::Result::Busy:
            return str << "Result Busy";
        case Ftp::Result::FileIoError:
            return str << "Result File Io Error";
        case Ftp::Result::FileExists:
            return str << "Result File Exists";
        case Ftp::Result::FileDoesNotExist:
            return str << "Result File Does Not Exist";
        case Ftp::Result::FileProtected:
            return str << "Result File Protected";
        case Ftp::Result::InvalidParameter:
            return str << "Result Invalid Parameter";
        case Ftp::Result::Unsupported:
            return str << "Result Unsupported";
        case Ftp::Result::ProtocolError:
            return str << "Result Protocol Error";
        default:
            return str << "Unknown";
    }
}

} // namespace mavsdk