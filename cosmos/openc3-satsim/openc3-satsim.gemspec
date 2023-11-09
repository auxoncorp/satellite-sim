# encoding: ascii-8bit

spec = Gem::Specification.new do |s|
  s.name = 'openc3-satsim'
  s.summary = 'OpenC3 satsim Targets'
  s.description = <<-EOF
    This plugin adds the OpenC3 satsim configuration.
  EOF
  s.authors = ['Jon Lamb']
  s.email = ['jon@auxon.io']
  s.homepage = 'https://auxon.io/'

  s.platform = Gem::Platform::RUBY

  if ENV['VERSION']
    s.version = ENV['VERSION'].dup
  else
    time = Time.now.strftime("%Y%m%d%H%M%S")
    s.version = '0.0.0' + ".#{time}"
  end
  s.licenses = ['Nonstandard']

  s.files = Dir.glob("{targets,lib,tools,microservices}/**/*") + %w(Rakefile LICENSE.txt README.md plugin.txt)
end
