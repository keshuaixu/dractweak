from tabnanny import check
import crc

configuration = crc.Configuration(16, 0x1db7, init_value=0xffff)
crc_calculator = crc.CrcCalculator(configuration)
# data = [0xAC, 0x45, 0x0F, 0x28]
# data.reverse()
data = [0x12] * 48 * 4
data = bytes(data)
checksum = crc_calculator.calculate_checksum(data)
print(hex(checksum))

