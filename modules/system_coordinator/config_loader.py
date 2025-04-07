"""
Configuration loader for the AMR system.
Handles parsing of INI configuration files with type conversion.
"""
import os
import configparser
import logging

logger = logging.getLogger(__name__)

class ConfigLoader:
    """Load and manage configuration from INI files."""
    
    def __init__(self, config_file):
        """
        Initialize the configuration loader.
        
        Args:
            config_file: Path to the configuration file
        """
        self.config_file = config_file
        self.config = configparser.ConfigParser()
        
        # Load configuration
        if os.path.exists(config_file):
            self.config.read(config_file)
            logger.info(f"Configuration loaded from {config_file}")
        else:
            logger.warning(f"Configuration file not found: {config_file}")
    
    def get_str(self, section, option, default=None):
        """
        Get a string value from the configuration.
        
        Args:
            section: Configuration section
            option: Configuration option
            default: Default value if option is not found
            
        Returns:
            String value
        """
        if section in self.config and option in self.config[section]:
            return self.config[section][option]
        return default
    
    def get_int(self, section, option, default=0):
        """
        Get an integer value from the configuration.
        
        Args:
            section: Configuration section
            option: Configuration option
            default: Default value if option is not found
            
        Returns:
            Integer value
        """
        if section in self.config and option in self.config[section]:
            try:
                return self.config.getint(section, option)
            except ValueError:
                logger.warning(f"Invalid integer value for {section}.{option}")
                return default
        return default
    
    def get_float(self, section, option, default=0.0):
        """
        Get a float value from the configuration.
        
        Args:
            section: Configuration section
            option: Configuration option
            default: Default value if option is not found
            
        Returns:
            Float value
        """
        if section in self.config and option in self.config[section]:
            try:
                return self.config.getfloat(section, option)
            except ValueError:
                logger.warning(f"Invalid float value for {section}.{option}")
                return default
        return default
    
    def get_bool(self, section, option, default=False):
        """
        Get a boolean value from the configuration.
        
        Args:
            section: Configuration section
            option: Configuration option
            default: Default value if option is not found
            
        Returns:
            Boolean value
        """
        if section in self.config and option in self.config[section]:
            try:
                return self.config.getboolean(section, option)
            except ValueError:
                logger.warning(f"Invalid boolean value for {section}.{option}")
                return default
        return default
    
    def get(self, section, option, type_hint=None, default=None):
        """
        Get a value from the configuration with type conversion.
        DEPRECATED: Use the specific methods (get_str, get_int, etc.) instead.
        
        Args:
            section: Configuration section
            option: Configuration option
            type_hint: Type hint ('str', 'int', 'float', 'bool')
            default: Default value if option is not found
            
        Returns:
            Value of the specified type
        """
        logger.warning("The get() method is deprecated. Use get_str, get_int, get_float, or get_bool instead.")
        
        if type_hint == 'str':
            return self.get_str(section, option, default)
        elif type_hint == 'int':
            return self.get_int(section, option, default)
        elif type_hint == 'float':
            return self.get_float(section, option, default)
        elif type_hint == 'bool':
            return self.get_bool(section, option, default)
        
        # No type hint or unrecognized type hint, return as string
        return self.get_str(section, option, default)
    
    def set(self, section, option, value):
        """
        Set a configuration value.
        
        Args:
            section: Configuration section
            option: Configuration option
            value: Value to set
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Ensure section exists
            if section not in self.config:
                self.config[section] = {}
            
            # Set option
            self.config[section][option] = str(value)
            
            # Save to file
            with open(self.config_file, 'w') as config_file:
                self.config.write(config_file)
            
            return True
        except Exception as e:
            logger.error(f"Error setting configuration value: {e}")
            return False
    
    def save(self):
        """
        Save configuration to file.
        
        Returns:
            True if successful, False otherwise
        """
        try:
            with open(self.config_file, 'w') as config_file:
                self.config.write(config_file)
            
            logger.info(f"Configuration saved to {self.config_file}")
            return True
        except Exception as e:
            logger.error(f"Error saving configuration: {e}")
            return False